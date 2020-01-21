// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h> //so we can call the qdesired

#include <controller_interface/multi_interface_controller.h> //to use multiple template in the class definition

#include <sensor_msgs/JointState.h>//for the callback, it can read the topic

//ROS message
#include <franka_msgs/FrankaState.h>


namespace panda_controllers {

class PdController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface> {
  
 public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time&);
  void stopping(const ros::Time&);
  void update(const ros::Time&, const ros::Duration& period);

 private:
   
  //topic's command listened 
  ros::Subscriber sub_command_;
  
  //variables used in updating
  double error, old_error,dot_error,commanded_effort,dt; //definition of the errors and the variable for the commanded effort 
  double Kp, Kv; //let's define the position and velocity gains
  
  //Command setting Callback
  void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);
  sensor_msgs::JointState command_; //definition of the desiderd position 
  
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; //used for the state of the joints and command the torque
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;  
 
  /*unique_ptr----->These objects have the ability of taking ownership of a pointer: once they take ownership 
   *they manage the pointed object by becoming responsible for its deletion at some point.*/
  
  
};

}  // namespace panda_controllers