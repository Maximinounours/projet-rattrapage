# example_publisher.launch.py

Example to publish a message to the robot.

Launch file to start the Gazebo simulation, the bridge with ROS2, and the publisher node:

```shell
ros2 launch mobile_robot_maxime example_publisher.launch.py
```

# secured_node_usr.launch.py

Example to pilot the robot with a CLI custom velocity (X linear and Z angular).

Launch file to start the Gazebo simulation, the bridge with ROS2, and the node:
```shell
ros2 launch mobile_robot_maxime secured_node_usr.launch.py
```

Send a velocity command:
```shell
ros2 run mobile_robot_maxime send_cmd_vel linear_x angular_z
```