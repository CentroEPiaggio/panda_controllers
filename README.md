# panda_controllers
Package of controllers implemented within the ROS Control framework for the Franka Emika Panda robot + elastic wrist + softhand.


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
   git clone --branch franka_wrist_softhand https://github.com/CentroEPiaggio/panda_controllers.git
   ```

1. Compile the package using `catkin`:
   ```
   cd ~/catkin_ws
   catkin build
   ```   
## Requirements 
Since you are interested in using this controllers you need to install libfranka and franka_ros, availables at: [franka_FCI](https://frankaemika.github.io/docs/)

## Running controllers
TO-DO
