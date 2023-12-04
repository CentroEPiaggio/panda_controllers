# panda-controllers - [project-adaptive](https://github.com/CentroEPiaggio/panda_controllers/tree/project_adaptive)

![Backstepping Adaptive](./image/project_adaptive_script.gif)

Here you can find backstepping adaptive controller and some classes implemented with casadi library to generalize adaptive control.

## Table of Contents
1. [Requirements](#requirements)
1. [Utils Adaptive](#utils-adaptive)
1. [Installation](#installation)
   1. `project-adaptive`
   1. `casadi`
   1. `yaml-cpp`
1. [ThunderPanda](#thunderpanda)
1. [Running controllers](#running-controllers)

## Requirements
* To test controllers in gazebo and/or RViz you need to install the package [franka_ros](https://github.com/frankaemika/franka_ros.git) in your workspace.
* If you want to use and/or modify implemented classes you have to install [casadi](https://github.com/casadi/casadi.git).
* If you want generate yaml file you have to install [yaml-cpp](https://github.com/jbeder/yaml-cpp.git).

## Utils Adaptive
The core of our project is in the folder [utils_adaptive](./utils_adaptive), here we created 4 main folders:

* `genThunder`: we implemented some classes, keeping in mind object-oriented paradigm, to generalize the approach to serial manipulator control, under the point of view of Denavit-Hartenberg parametrization. Important functions were obtainable only with the support of casadi library (for example Christoffel symbols). In particular the aim of this folder is to generate fast and efficient code for Franka Emika Panda to satisfy the constraints of real time.
* `genYAML`: we manipulate "original" Franka Emika Panda inertial parameters in `inertial.yaml` to create yaml files for Denavit-Hartenberg parametrization and then for the Regressor.
* `generatedFiles`: it contains the files generated from 2 folders above.
* `chrono_test`: it contains the files to compare execution-time of different algorithms.

## Installation
Steps for installation:

### project-adaptive - from source

1. Clone the `panda_controllers` branch `project-adaptive` to your Catkin Workspace, e.g. `~/catkin_ws`:
   ```
   cd ~/catkin_ws/src
   git clone --branch project-adaptive https://github.com/CentroEPiaggio/panda_controllers.git
   ```

1. Compile the package using `catkin`:
   ```
   cd ~/catkin_ws
   catkin_make
   ```

### casadi - from source
   
Before continue check the instructions of installation in the official [API site](https://casadi.sourceforge.net/api/html/d3/def/chapter2.html), [git repostery](https://github.com/casadi/casadi.git) and [website](https://web.casadi.org/).

That are following:

1. Clone the `main` repostery of `casadi`:
   ```
   git clone https://github.com/casadi/casadi.git -b main casadi
   ```

1. Set the environment variable `CMAKE_PREFIX_PATH` to inform CMake where the dependecies are located. For example if headers and libraries are installed under `$HOME/local/`, then type:
   ```
   export CMAKE_PREFIX_PATH=$HOME/local/
   ```

1. Make install
   ```
   cd casadi; mkdir build; cd build
   cmake ..
   ccmake ..
   make
   make install
   ```

### yaml-cpp - from .zip

Before continue follow installation instruction from repostery https://github.com/jbeder/yaml-cpp.

That are following:

1. Download local clone .zip from https://github.com/jbeder/yaml-cpp and extract.
1. Navigate into the source directory and run:
   ```
   mkdir build; cd build
   cmake ..
   cmake --build .
   make
   sudo make install
   ```
## ThunderPanda

[ThunderPanda](./src/) is a class useful for implementing controllers without the dependecy of casadi library. Allocate a ThunderPanda object in your code, and you have access to all matrices for kinematic and dynamic of the robot implemeted in utils adaptive classes.

## Running Controllers

* In [launch](./launch/) folder there is a launch file related to the spawning of controller `panda_controllers_sim.launch`, that is useful to set type of controller in gazebo and rviz visualization. If you want set up controllores in real robot you have two option: `backstepping_controller.launch` or `computed_torque_controller.launch`.

* In [config](./config/) folder you can use `homing.yaml` to set homing value of joints that can be used in 
`homing.launch`. You can set parameters of controllers in `panda_controllers_default.yaml` and parameters of some trajectories in `trajectory_param.yaml` used in `command.launch`.

The [launch_map.pdf](./launch_map.pdf) file is a simple map of the launch files and flag and parameters that can be manipulated.