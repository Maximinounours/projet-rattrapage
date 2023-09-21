from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

def generate_launch_description():
    return LaunchDescription([
        # Setup env variables for the .sdf
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=os.getcwd() + '/src/mobile_robot_maxime/models'),

        # Start simulation
        ExecuteProcess(cmd=['ign','gazebo', './src/mobile_robot_maxime/worlds/simulation.sdf']),

        # Start all bridges
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
                            ]),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_joint/_0@std_msgs/msg/Float64]ignition.msgs.Double'
                            ]),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_joint/_1@std_msgs/msg/Float64]ignition.msgs.Double'
                            ]),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_joint/_2@std_msgs/msg/Float64]ignition.msgs.Double'
                            ]),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_joint/_3@std_msgs/msg/Float64]ignition.msgs.Double'
                            ]),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_joint/_4@std_msgs/msg/Float64]ignition.msgs.Double'
                            ]),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/cmd_joint/_5@std_msgs/msg/Float64]ignition.msgs.Double'
                            ]),
        ExecuteProcess(cmd=['ros2',
                            'run',
                            'ros_ign_bridge',
                            'parameter_bridge',
                            '/world/simple_world/model/robot/model/ur5/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
                            ]),
    ])
