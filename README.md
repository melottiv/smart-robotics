<p align="center">
  <h2 align="center">Smart Robotics Project: burgerBot </h2>

  
</p>
<br>

<img src="https://github.com/melottiv/smart-robotics/blob/frenci/main.png">

## Table of contents
- [Description](#description)
- [Requirement](#requirements)
- [Folder](#folder)
- [Setup](#setup)
- [Usage](#usage)
- [Contributors](#contributors)

### Description
This repository demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a Xbox Kinect cam to detect different ingredients, and publish their position and angolation. 

The goals of this project are:
- simulate the iteration of a UR5 robot with simple shapes, meant to stand in for ingredients
- The robotic arm must be able to detect ingredients and follow user specifications to stack them into a burger

<img src="https://github.com/melottiv/smart-robotics/blob/frenci/intro.gif">

### Folder
```
FactoryNinja-precision-sorting-to-get-the-chaos-off/catkin_ws/src
├── ninja_manager
├── vision
├── motion_planning
├── gazebo_ros_link_attacher
├── robot
```
- `ninja_manager:` the task of this package is to launch the world and spawn all the ingredients
- `vision:` the task of this package is to recognize the ingredients based on its shape and color and localize it
- `motion_planning:` the task is to move the robot and pick and place the ingredient element, after interaction with the user
- `gazebo_ros_link_attacher:` A gazebo plugin definable from URDF to inform a client of a collision with an object
- `robot:` the task is to define the robot model with appropriate PID settings


### Requirements

For running each sample code:
- `Ros Noetic:` http://wiki.ros.org/noetic/Installation
- `Gazebo:` https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
- `Catkin` https://catkin-tools.readthedocs.io/en/latest/

### Setup

After installing the libraries needed to run the project. Clone this repo:
```
git clone https://github.com/melottiv/smart-robotics
```

Setup the project:
```
cd smart-robotics/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
echo "source $PWD/devel/setup.bash" >> $HOME/.bashrc
```

### Usage

Launch the world
```
roslaunch ninja_manager ninja_world.launch
```
Spawn all the ingredients
```
rosrun ninja_manager ninja-manager.py 
```
Start the kinematics process
```
rosrun motion_planning motion_planning.py
```
Start the localization process
```
rosrun vision ninja-vision.py -show
```
- `-show` : show the results of the recognition and localization process with an image

### Contributor

| Name                 | Github                                                          |
|----------------------|-----------------------------------------------------------------|
| Melotti Virginia 	| [melottiv](https://github.com/melottiv)				|
| Morandi Francesca 	| [francymory](https://github.com/francymory)				|

