from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os
import xacro
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    ign_verbosity = LaunchConfiguration("ign_verbosity")

    # set env variable to find the robot model in IGN
    env_var = [
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=os.getcwd() + "/src/robot_description/resources",
        )
    ]

    # Package Directories
    pkg_description = get_package_share_directory("ur_no_ee_moveit_config")

    # Parse robot description from xacro
    robot_description_file = os.path.join(
        pkg_description, "config", "ur5_rg2.urdf.xacro"
    )
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Start gazebo with default world from ros_ign_gazebo package
    launch_descriptions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_ign_gazebo"),
                        "launch",
                        "ign_gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments=[("ign_args", [world, " -r -v ", ign_verbosity])],
        )
    ]

    nodes = [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "myrobot", "-topic", "robot_description"],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
    ]

    return LaunchDescription(declared_arguments + env_var + nodes + launch_descriptions)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World for Ignition Gazebo
        # Default world from ros_ign_gazebo package
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="4",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
    ]
