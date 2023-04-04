# panda_controllers

Package of controllers (cartesian impedance and computed torque) implemented within the ROS Control framework for the Franka Emika Panda robot.

## Table of Contents
1. [Installation](#markdown-header-installation)
   1. [Installation from Source](#markdown-header-from-source)
1. [Requirements](#markdown-header-requirements)
1. [Run controllers](#markdown-header-overviews)

Steps for installation:

1. Clone the `soft_wrist` package to your Catkin Workspace, e.g. `~/catkin_ws`:
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/CentroEPiaggio/panda_wrist_softhand.git 
   ```

1. Compile the package using `catkin`:
   ```
   cd ~/catkin_ws
   catkin_make
   ```
   **Note:** depending on your ROS installation, you may need some extra packages to properly compile the code.
