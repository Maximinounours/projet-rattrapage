# Gazebo ROSCON2022 notes

## ROS2 & Gazebo integration best practices

https://vimeo.com/767127300

### Choose ROS2 & Gazebo versions

- Now is good to migrate to ROS Humble & Gazebo Fortress/Garden
- Humble + Fortress = binaries available
- Humble + Garden = only from source

ROS2 users mostly use Gazebo Fortress.

### Structure project

Template: ros_gz_project_template (https://github.com/gazebosim/ros_gz_project_template.git) (Humble + Garden)

Bunch of systems already implemented: https://github.com/gazebosim/gz-sim

### Entity Component Manager

- Everything in the simulation is an entity (link, joint, model, world, visual, etc.)
- Each entity can have 1 or more components (velocity, name, pose, etc.)

### Writing Gazebo Systems

When you write a Gazebo system, many update points during a simulation
- **Configuration**: plugin load, ECM and DSF attributes
- **preupdate**: mutate components to set forces, torques, etc.
- **update**: physics update
- **postupdate**: read components, publish/send events, read sensors, properties of the world, etc.
- **reset**

### Making a Gazebo system with ROS2

Gazebo systems are shared libraries located via environment variables

with ROS2, ament_hooks to install and locate Gazebo systems

CMakeLists.txt only ?

### Connect Gazebo and ROS2

2 mechanisms:

**ros_gz_bridge to connect topics between ROS2 and Gazebo**

YAML file to configure it (or command line)
- specify ROS2 topic names, Gazebo topic names, types, direction (ROS to GZ or GZ to ROS), lazey subscribers/publishers, etc.

- Everything is topics and services for the communication

**Embedding ROS2 in GZ**

C++ ? Couples GZ and ROS2

### Simulation Assets

Assets = models (URDF, SDF, etc.), worlds, materials, etc.

Can be installed as part of ROS2 packages and exported as model://

They are SDF files, but support other formats (URDF)

Can view and use models: app.gazebosim.org/dashboard (open source assets)

To use assets: publish sdf file in robot description topics, and then it finds which plugin and thats it?

Can have the same model simulated in Rviz and Gazebo.

### bilan

Check the template

### Template: missing packages

- https://github.com/ros/sdformat_urdf