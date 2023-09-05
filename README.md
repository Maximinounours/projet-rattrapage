# Tutorial ROS2

https://docs.ros.org/en/humble/Tutorials.html

# Package

Make a workspace with a src folder
```shell
mkdir -p ./myworkspace/src
cd ./myworkspace/src
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

- Add a bunch of cool files for nodes actions topics and such

- Then build your package from the workspace
```shell
colcon build --packages-select my_package
```

- Source it
```shell
source install/local_setup.bash
```

- And run it
```shell
ros2 run my_package my_node
```

You can modify the description and such in `my_package/package.xml` **BUT YOU NEED TO MATCH THE CHANGES IN** `my_package/setup.py`

