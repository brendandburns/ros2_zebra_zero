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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_file = os.path.join(
            get_package_share_directory('zebra_zero'),
            'urdf',
            'arm.urdf')

    robot_description_content = open(robot_description_file,'r').read()
 
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("zebra_zero"),
            "config",
            "zebra_zero.yaml",
        ]
    )
#    rviz_config_file = PathJoinSubstitution(
#        [FindPackageShare("ros2_control_demo_description"), "r6bot/rviz", "view_robot.rviz"]
#    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # parameters=[robot_description, robot_controllers],
        parameters=[robot_controllers],
        remappings=[
            ("/forward_position_controller/commands", "/position_commands"),
            # See https://github.com/ros-controls/ros2_control/issues/1262
            ("~/robot_description", "robot_description"),
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
#    rviz_node = Node(
#        package="rviz2",
#        executable="rviz2",
#        name="rviz2",
#        output="log",
#        arguments=["-d", rviz_config_file],
#    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zebra_zero", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
#    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
#        event_handler=OnProcessExit(
#            target_action=joint_state_broadcaster_spawner,
#            on_exit=[rviz_node],
#        )
#    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
#        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
