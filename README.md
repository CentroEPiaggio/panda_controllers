# panda-controllers - [project-adaptive](https://github.com/CentroEPiaggio/panda_controllers/tree/project_adaptive)

![Backstepping Adaptive](./image/project_adaptive_script.gif)

Here you can find backstepping adaptive controller and some classes implemented with casadi library to generalize adaptive control.

## Table of Contents
1. [Requirements](#Requirements)
1. [Utils Adaptive](#Utils-Adaptive)
1. [Installation](#Installation)
   1. `project-adaptive`
   1. `casadi`
   1. `yaml-cpp`
1.[ThunderPanda](#ThunderPanda)
1. [Running controllers](#Running-controllers)

## Requirements
* To use this controllers you need to install the package [franka_ros](https://github.com/CentroEPiaggio/franka_ros.git) in your workspace.
* If you want use and/or modify implemented casadi classes you have to install [casadi](https://github.com/casadi/casadi.git).
* If you want generate yaml file you have to install [yaml-cpp](https://github.com/jbeder/yaml-cpp.git).

## Utils Adaptive
The core of our project is in the folder [utils_adaptive](./utils_adaptive), here we created 3 main folders:

* `genThunder`: we implemented vary classes, keeping in mind object-oriented paradigm to generalize the approach to serial manipulator control, under the point of view of Denavit-Hartenberg parametrization. Important functions were obtainable only with the support of casadi library (for example Christoffel symbols). In particular the aim of this folder is to generate fast and efficient code for Franka Emika Panda to satisfy the constraints of real time.
* `genYAML`: we manipulate "original" Franka Emika Panda inertial parameters in `inertial.yaml` to create yaml files for Denavit-Hartenberg parametrization and then for Regressor.
* `generatedFiles`: it contains the files generated from 2 folders above. 

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

[ThunderPanda](./panda_controllers/src/) is a class useful for implement controllers without casadi library. Allocate a ThnederPanda object in your code, and you have access to all matrixes for kinematic and dynamic of the robot implemeted in utils adaptive classes.

## Running Controllers

In the package, there is a launch file related to the spawning of controller `panda_controllers_sim.launch`, that is useful to set type of controller and rviz visualization.
There are already some simple trajectory implemented that are defined in the yaml file `trajectory_param.yaml` that are used in node `command_cartesian.cpp`, runnable with launch file `command.launch`.