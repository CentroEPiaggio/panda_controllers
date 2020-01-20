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

#include <controller_interface/multi_interface_controller.h>

#include <sensor_msgs/JointState.h>

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
  
  //variables
  double error, old_error,dot_error,commanded_effort,dt;
   
  //Command setting Callback
  void setCommandCB(const sensor_msgs::JointStateConstPtr &msg);
  sensor_msgs::JointState command_; //definition of the desiderd position 
  double Kp, Kv; //let's define the position and velocity gains
  hardware_interface::JointHandle joint_; //Selection of the joints
  
  ros::Duration elapsed_time_; //?
};

}  // namespace panda_controllers