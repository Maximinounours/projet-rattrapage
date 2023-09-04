# Tutorial ROS2

https://docs.ros.org/en/humble/Tutorials.html

### Configuring environment

Tout est dans le .bashrc
- Source le fichier setup
- Setup DOMAIN_ID à 0
- ROS_LOCALHOST_ONLY à 1

### Using turtlesim, ros2, and rqt

Start :
```shell
ros2 run turtlesim turtlesim_node
```
Interact:

```shell
ros2 run turtlesim turtle_teleop_key
```

### Understanding nodes

Un node s'occupe d'une action (controle roue, recup info capteur, ...)

Les nodes peuvent recevoir et envoyer des donnes via des services, topics, actions, ou parameters.

Remapping : pour modifier des params d'un node (nom, topic, service, ...)

### Understanding topics

- Pour avoir le graph des topics nodes, etc.
```shell
rqt
```

Un topic peut servir de multiplexeur / demultiplexeur pour transmettre des infos.

Les nodes qui envoient un msg sont des **publishers**, ceux qui recoivent un message sont les **subscribers**.

- Check les messages qui circulent dans un topic avec `ros2 topic echo` :
```shell
ros2 topic echo /turtle1/cmd_vel (pour le topic cmd_vel)
```

- Savoir les subscribers/publishers et types de message d'un topic :
```shell
ros2 topic info /turtle1/cmd_vel
```

- Détail de la structure d'un message avec `ros2 interface show`
```shell
ros2 interface show geometry_msgs/msg/Twist
```

- Directement publier un message (bien pour tester) avec `ros2 topic pub`
```shell
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Ou plutot que de communiquer l'information une seule fois `--once` on peut utiliser `--rate` en Hz

```shell
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

### Understanding services

Système de requete / reponse pour les messages.\
Topics sont la pour se connecter directement au flux, services ne communiquent que si on le demande.

- Obtenir info sur un service avec `ros2 service type`
```shell
ros2 service type /clear
# Ou directement
ros2 service list -t
```

- Savoir les entrées / sorties d'un service avec `ros2 interface show`
```shell
ros2 interface show turtlesim/srv/Spawn
```
- De la même manière que les topics, on peut call un service `ros2 service call`
```shell
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

### Understanding parameters

Parameters sont des config pour les nodes

- Pour les voir oragnisés par nodes
```shell
ros2 param list
```

- Pour avoir la valeur d'un param
```shell
ros2 param get /turtlesim background_g
```

- Et pour la set
```shell
ros2 param set /turtlesim background_g 150
```

- Get all params at once with their values
```shell
ros2 param dump /turtlesim
```
And quick tips to save all states, dump the stdout to a file by adding `> turtlesim.yaml` at the end.
You can then load the params with :
```shell
ros2 param load /turtlesim turtlesim.yaml
```

- These params can be used when starting the whole node :
```shell
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

### Understanding actions

Action are for long running tasks and consists in **goal, feedback, result**.

functionality similar to services, but provide steady feedback (services are 1 time only), and can be canceled.

Use client-server system (like topics). Client sends a goal, server acknowledges goal, sends (a stream of) feedback, and sends result.

- List of the actions available
```shell
ros2 action list -t
```

- The action server and action client can be found the action info
```shell
ros2 action info /turtle1/rotate_absolute
```

- And to know the action structure call (goal --- result --- feedback)
```shell
ros2 interface show turtlesim/action/RotateAbsolute
```

- To send a goal to an action, with some feedback :
```shell
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

### Using rqt_console to view logs

This is to inspect log messages.
```shell
ros2 run rqt_console rqt_console
```

- Possibility to change the minimum message level
```shell
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```

### Launching nodes

You can use a start file to launch all nodes with their configs at once.

```shell
ros2 launch turtlesim multisim.launch.py
```

With
```python
# turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
    ])
```

### Recording and playing back data

- You can save in a file and play back everything that happens in a topic

```
ros2 bag record /turtle1/cmd_vel
```

- Or multiple topics (in the file subset) (`-a` option for all)
```shell
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

- And replay with

```shell
ros2 bag play subset
```

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