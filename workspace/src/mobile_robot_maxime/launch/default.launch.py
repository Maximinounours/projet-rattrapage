import os
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from typing import List
from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory

PKG_NAME = "mobile_robot_maxime"


def generate_launch_description():
    """Launches Gazebo simulation with an empty world"""
    ld = LaunchDescription()

    # --- ENV VARIABLES ---
    # Set environment variables to find model meshes
    env_variable_gazebo = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.join(get_package_share_directory("robot_description"), "models"),
    )
    ld.add_action(env_variable_gazebo)

    # --- LAUNCH PARAMS ---

    # Get declared params
    declared_args = generate_declared_arguments()
    world = LaunchConfiguration("world")
    robot = LaunchConfiguration("robot")

    # Have to add each element individually
    for arg in declared_args:
        ld.add_action(arg)

    # --- START SIM ---

    # Launch Ignition Gazebo with world given
    launch_world = IncludeLaunchDescription(
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
    )

    ld.add_action(launch_world)

    # --- SPAWN ENTITIES ---

    # Spawn robot
    node_spawn = Node(
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
    ld.add_action(node_spawn)

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
