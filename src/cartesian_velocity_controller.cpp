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

#define   DEBUG_CVC   0       // Additional printouts

namespace panda_controllers {

bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hardware,
  ros::NodeHandle& node_handle) {

  // Initializing node handle
  this->cvc_nh = node_handle;
  
  // Getting arm id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id.");
    return false;
  }

  // Getting command timeout
  double comm_timeout;
  if (!node_handle.getParam("command_timeout", comm_timeout)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter command_timeout.");
    return false;
  }
  this->command_timeout = ros::Duration(comm_timeout);

  // Getting cartesian velocity limits
  if (!node_handle.getParam("velocity_thresh/translation", this->translation_limit)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter velocity_thresh/translation.");
    return false;
  }
  if (!node_handle.getParam("velocity_thresh/rotation", this->rotation_limit)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter velocity_thresh/rotation.");
    return false;
  }
  if (!node_handle.getParam("velocity_thresh/elbow", this->elbow_limit)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter velocity_thresh/elbow.");
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

  // Setting up the filter (This is needed here otherwise when switching controllers, filter memory is not cleared!)
  this->vel_filter.clear();
  for(int i = 0; i < 6; i++){
    this->vel_filter.push_back(std::make_shared<filters::FilterChain<double>>("double"));
    this->vel_filter[i]->configure("low_pass_filter", this->cvc_nh);
  }
  
  // Setting the last recieved time
  this->last_command_time = ros::Time::now();

  // Setting the value of the velocity command to zero
  this->vel_mutex.lock();
  std::fill(std::begin(this->vel_command), std::end(this->vel_command), double(0.0));
  this->vel_mutex.unlock();
}

void CartesianVelocityController::update(const ros::Time& /* time */, const ros::Duration& period) {
  
  // Resetting command if timeout
  if(ros::Time::now() - this->last_command_time > this->command_timeout){
    this->vel_mutex.lock();
    std::fill(std::begin(this->vel_command), std::end(this->vel_command), double(0.0));
    this->vel_mutex.unlock();
  }

  // Filtering the requested command
  this->vel_mutex.lock();
  for(int i = 0; i < 6; i++){
    this->vel_filter[i]->update(this->vel_command[i], this->filt_command[i]);
  }
  this->vel_mutex.unlock();

  if(DEBUG_CVC){
    std::cout << "The commanded twist is [ ";
    for(auto comm_print : this->vel_command){
      std::cout << comm_print << " "; 
    }
    std::cout << "]" << std::endl;
    std::cout << "The filtered twist is [ ";
    for(auto comm_print : this->filt_command){
      std::cout << comm_print << " "; 
    }
    std::cout << "]" << std::endl;
  } 

  // Setting the filtered commanded twist
  this->velocity_cartesian_handle_->setCommand(this->filt_command);
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {

  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.

  // Setting the value of the velocity command to zero
  this->vel_mutex.lock();
  std::fill(std::begin(this->vel_command), std::end(this->vel_command), double(0.0));
  this->vel_mutex.unlock();
}

// Command setting callback
void CartesianVelocityController::command(const geometry_msgs::Twist::ConstPtr &msg){
  
  // Setting the last recieved time
  this->last_command_time = ros::Time::now();

  // If there are nans or infs in the command, set to zero
  bool all_finite = this->check_if_finite(*msg);

  // Cutting off the command to cartesian velocity limits
  // TODO if necessary

  // Converting and saving msg to command (TODO: check variation)
  this->vel_mutex.lock();
  // Check if franka is ok, if so set command else set zero velocities
  if(this->franka_ok && all_finite){
    this->vel_command = {{msg->linear.x, msg->linear.y, msg->linear.z,
      msg->angular.x, msg->angular.y, msg->angular.z}};
  } else {
    if (!all_finite) {
      ROS_WARN_STREAM("panda_controllers::CartesianVelocityController : The requested velocity vector contains NaNs or Infs, setting x_ref to null.");
    }
    std::fill(std::begin(this->vel_command), std::end(this->vel_command), double(0.0));
  }
  this->vel_mutex.unlock();
}

// Franka states emergency callback
void CartesianVelocityController::get_franka_states(const franka_msgs::FrankaState::ConstPtr &msg){

  // See if robot got into error mode
  if(msg->robot_mode != 2 && msg->robot_mode != 5){       // The robot state is not "automatic" or "manual guiding"
      this->franka_ok = false;
      if(this->print_once){
        ROS_ERROR("Robot entered in error mode!");
        this->print_once = false;
      }
  }else if(msg->robot_mode == 2){
      this->franka_ok = true;
      this->print_once = true;
  }

  // See if some violation occured without getting into errors
  this->franka_ok = (msg->current_errors.joint_position_limits_violation || msg->current_errors.joint_velocity_violation
    || msg->current_errors.joint_reflex || msg->current_errors.cartesian_reflex || msg->current_errors.tau_j_range_violation);
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::CartesianVelocityController, controller_interface::ControllerBase)
