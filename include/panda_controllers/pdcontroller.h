// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>


//ROS message
#include <franka_msgs/FrankaState.h>


namespace panda_controllers {

class PdController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
  
 public:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& n) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
   
  //topic's command listened 
  ros::Subscriber sub_command_;
   
  //Command setting Callback
  void setCommandCB(const std_msgs::Float64ConstPtr& msg)
  double command_; //definition of the desiderd position 
  double Kp, Kv; //let's define the position and velocity gains
  hardware_interface::JointHandle joint_; //Selection of the joints
  
  ros::Duration elapsed_time_; //?
};

}  // namespace panda_controllers