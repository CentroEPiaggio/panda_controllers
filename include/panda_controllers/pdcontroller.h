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
#include <eigen3/Eigen/Dense>

#include <Eigen/Dense>
using namespace Eigen;


namespace panda_controllers {

class PdController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface> {
  
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
  void starting(const ros::Time&);
  void stopping(const ros::Time&);
  void update(const ros::Time&, const ros::Duration& period);

 private:
   
  //topic's command listened 
  ros::Subscriber sub_command_;
  
  //Defining variables
  double error, old_error,dot_error,commanded_effort,dt; //definition of the errors and the variable for the commanded effort 
  
  /* Defining Position and Velocity Gains */
  Eigen::DiagonalMatrix<double, 7> kp;
  Eigen::DiagonalMatrix<double, 7> kv;
  
  /* Defining q_current, q_current_dot, q_des, and tau_cmd*/
  
  Eigen::Matrix<double, 7, 1> q_curr;
  Eigen::Matrix<double, 7, 1> dq_curr;
  Eigen::Matrix<double, 7, 1> q_des; ///// dovrei pescarla dal command_.....c'è da fare una conversione di data type
  Eigen::Matrix<double, 7, 1> tau_cmd;
  
  
  
  //Command setting Callback
  
  void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);
  
  
  Eigen::Matrix<double, 7, 1> command_pos; //definition of the desiderd position 
  Eigen::Matrix<double, 7, 1> command_dot_pos;
  
  /* DUBBIO: E' lecito fare una cosa del genere "std::array<sensor_msgs::JointState, 7> pippo ??????????*/
  
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; //used for the state of the joints and command the torque
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;  
 
  /*unique_ptr----->These objects have the ability of taking ownership of a pointer: once they take ownership 
   *they manage the pointed object by becoming responsible for its deletion at some point.*/
  
  
};

}  // namespace panda_controllers