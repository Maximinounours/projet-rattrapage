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
ros2 service call /clear std_srvs/srv/Empty
```