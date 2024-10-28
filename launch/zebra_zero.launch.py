# Derived from https://github.com/ros-controls/ros2_control_demos/blob/humble/example_7/bringup/launch/r6bot_controller.launch.py
# License preserved from there:
#
# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    def create_robot_description(context): 
        robot_description_file = os.path.join(
            get_package_share_directory('zebra_zero'),
            'urdf',
            'arm.urdf.xacro')

        mappings = {
            'ros2_control_hardware_type': context.launch_configurations['ros2_control_hardware_type']
        }
        document = xacro.process_file(robot_description_file, mappings=mappings)
        robot_description_content = document.toprettyxml(indent="  ")

        return [SetLaunchConfiguration('robot_desc', robot_description_content)]

    create_robot_description_arg = OpaqueFunction(function=create_robot_description)
    robot_description = {"robot_description": LaunchConfiguration('robot_desc')}
 
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("zebra_zero"),
            "config",
            LaunchConfiguration("controller_config"),
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("/forward_position_controller/commands", "/position_commands"),
            # See https://github.com/ros-controls/ros2_control/issues/1262
            ("~/robot_description", "robot_description"),
        ],
        output="both",
        arguments=['--ros-args', '--log-level', ['ZebraZeroHardware', ':=', LaunchConfiguration('log_level')]]
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zebra_zero",
                   "velocity_controller",
                   # "cartesian_motion_controller",
                   "zerog_controller",
                   "trajectory_controller",
                   # "motion_control_handle",
                   "zerog_controller",
                   "trajectory_controller",
                   "-c", "/controller_manager",
                   "--inactive"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        DeclareLaunchArgument(
            "log_level",
            default_value = TextSubstitution(text=str("INFO")),
            description="Logging level"
        ),
        DeclareLaunchArgument(
            "controller_config",
            default_value = TextSubstitution(text=str("zebra_zero.yaml")),
            description="Controller configuration"
        ),
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value = 'zebra',
            description='The type of hardware [mock_components, zebra]'
        ),
        create_robot_description_arg,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(
        nodes)
