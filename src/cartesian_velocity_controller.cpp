// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_controllers/cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_controllers {

bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hardware,
  ros::NodeHandle& node_handle) {
  
  // Getting needed parameters
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id.");
    return false;
  }
  
  // Get the cartesian velocity interface fro the robot hardware
  this->velocity_cartesian_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (this->velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR("CartesianVelocityController: Could not get cartesian velocity interface from hardware.");
    return false;
  }
  
  // Getting the related handle
  try {
    this->velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(this->velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianVelocityController: Error (" << e.what() << ") getting cartesian velocity handle.");
    return false;
  }

  // Initializing subscriber to command topic
  this->sub_command_ = node_handle.subscribe<geometry_msgs::Twist>("command", 1, &CartesianVelocityController::command, this);

  return true;
}

void CartesianVelocityController::starting(const ros::Time& /* time */) {
  
  // Nothing to do here
}

void CartesianVelocityController::update(const ros::Time& /* time */, const ros::Duration& period) {
  
  // Setting the commanded twist
  this->vel_mutex.lock();
  this->velocity_cartesian_handle_->setCommand(this->vel_command);
  this->vel_mutex.unlock();
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {

  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

// Command setting callback
void CartesianVelocityController::command(const geometry_msgs::Twist::ConstPtr &msg){
  
  // Converting and saving msg to command (TODO: check variation)
  this->vel_mutex.lock();
  this->vel_command = {{msg->linear.x, msg->linear.y, msg->linear.z,
    msg->angular.x, msg->angular.y, msg->angular.z}};
  this->vel_mutex.unlock();
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::CartesianVelocityController, controller_interface::ControllerBase)
