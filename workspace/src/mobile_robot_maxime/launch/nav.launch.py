import os
from typing import List
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

PKG_NAME = "mobile_robot_maxime"


def generate_launch_description():
    ld = LaunchDescription()

    # Get launch args
    declared_args = generate_declared_arguments()
    robot = LaunchConfiguration("robot")
    world = LaunchConfiguration("world")
    for arg in declared_args:
        ld.add_action(arg)

    # Start default.launch
    launch_descr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory(PKG_NAME),
                    "launch",
                    "default.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("world", world),
            ("robot", robot),
        ],
    )

    ld.add_action(launch_descr)

    # Start bridges to communicate between ros and gazebo
    bridge_cmd_vel = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "]" + "ignition.msgs.Twist",
        ],
    )
    ld.add_action(bridge_cmd_vel)

    bridge_odom = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            "/model/mobile_base/odometry"
            + "@nav_msgs/msg/Odometry"
            + "["
            + "ignition.msgs.Odometry",
        ],
    )
    ld.add_action(bridge_odom)

    # Start service node for navigation
    node_nav = Node(
        package="navigation_maxime",
        executable="compute_nav",
        output="screen",
    )
    ld.add_action(node_nav)

    return ld


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            "robot",
            default_value="mobile_base",
            description="Robot path or filename to spawn. If filename,"
            + "it must be in the directory specified by $IGN_GAZEBO_RESOURCE_PATH",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                get_package_share_directory("mobile_robot_maxime"),
                "worlds",
                "empty.sdf",
            ),
            description="World filename or path to load. If filename,"
            + "it must be in the directory specified by $IGN_GAZEBO_RESOURCE_PATH",
        ),
    ]
