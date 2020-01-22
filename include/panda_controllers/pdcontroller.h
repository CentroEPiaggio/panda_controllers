#pragma once

#include <array>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>

#include <controller_interface/multi_interface_controller.h> //To use multiple interface template in the class PdController definition

#include <controller_interface/controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h>

#include <sensor_msgs/JointState.h>

//ROS message
#include <franka_msgs/FrankaState.h>



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
  
  /* Defining Position and Velocity Gains */
  Eigen::MatrixXd Kp;
  Eigen::MatrixXd Kv;
  
  /* Defining q_current, q_current_dot, q_des, and tau_cmd*/
  
  Eigen::Matrix<double, 7, 1> q_curr;
  Eigen::Matrix<double, 7, 1> dq_curr; 
  Eigen::Matrix<double, 7, 1> tau_cmd;
  
  /* Setting Command Callback*/
  
  void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);
  
  Eigen::Matrix<double, 7, 1> command_pos; //definition of the desiderd position 
  Eigen::Matrix<double, 7, 1> command_dot_pos;
  
  
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; //used for the state of the joints and command the torque
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;  
 
  /*unique_ptr----->These objects have the ability of taking ownership of a pointer: once they take ownership 
   *they manage the pointed object by becoming responsible for its deletion at some point.*/
  
  
};

}  // namespace panda_controllers