from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=os.getcwd() + '/src/mobile_robot_maxime/models'),
        ExecuteProcess(cmd=['ign','gazebo', './src/mobile_robot_maxime/worlds/simulation.sdf']),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
                            ]),
        Node(package='mobile_robot_maxime',
              executable='velocity_pub',
 output='screen')
    ])