from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=os.getcwd() + "/src/mobile_robot_maxime/models",
            ),
            ExecuteProcess(
                cmd=["ign", "gazebo", "./src/mobile_robot_maxime/worlds/empty.sdf"]
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-file",
                    os.getcwd() + "/src/mobile_robot_maxime/models/robot/model.sdf",
                    "-name",
                    "test",
                    "-z",
                    "1",
                ],
            ),
        ]
    )
