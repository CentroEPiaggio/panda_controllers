# panda_controllers

Package of controllers (cartesian impedance and computed torque) implemented within the ROS Control framework for the Franka Emika Panda robot.

## Table of Contents
1. [Installation](#markdown-header-installation)
   1. [Installation from Source](#markdown-header-from-source)
1. [Requirements](#markdown-header-requirements)
1. [Running controllers](#markdown-header-overviews)

## Installation
### Installation from Source

Steps for installation:

1. Clone the `panda_controllers` package to your Catkin Workspace, e.g. `~/catkin_ws`:
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/CentroEPiaggio/panda_controllers.git 
   git checkout stefano-darko
   ```

1. Compile the package using `catkin`:
   ```
   cd ~/catkin_ws
   catkin_make
   ```   
## Requirements 
Since you are interested in using this controllers you need the following package in your workspace: [franka_ros](https://github.com/CentroEPiaggio/franka_ros.git)

## Running controllers

In the `panda_controllers` package, there are two launch file related to the spawning of the cartesian and computed torque controller,`cartesian_impedance_controller_softbots.launch` and `computed_torque_controller.launch` , respectively.
