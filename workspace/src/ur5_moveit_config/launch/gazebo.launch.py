#!/usr/bin/env -S ros2 launch
"""Launch default world with the default robot (configurable)"""

from typing import List
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    # List of included launch descriptions
    declared_arguments = generate_declared_arguments()

    world = LaunchConfiguration("world")
    robot = LaunchConfiguration("robot")

    # Start simulation
    launch_sim = [
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=os.path.join(
                get_package_share_directory("robot_description"), "models"
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        get_package_share_directory("ros_ign_gazebo"),
                        "launch",
                        "ign_gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments=[("ign_args", [world, " -r"])],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(["ur5_moveit_config"]),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("ros2_control_plugin", "ign"),
                ("ros2_control_command_interface", "effort"),
                # TODO: Re-enable colligion geometry for manipulator arm once spawning with specific joint configuration is enabled
                ("collision_arm", "true"),
            ],
        ),
    ]

    # Spawn robot
    launch_robot = [
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-file",
                robot,
                "-z",
                "0.5",
            ],
        )
    ]

    return LaunchDescription(declared_arguments + launch_sim + launch_robot)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            "robot",
            default_value=["mobile_arm"],
        ),
        DeclareLaunchArgument(
            "world",
            default_value=[
                os.path.join(
                    get_package_share_directory("mobile_robot_maxime"),
                    "worlds",
                    "empty.sdf",
                )
            ],
        ),
    ]
