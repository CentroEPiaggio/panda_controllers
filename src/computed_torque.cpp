//various library on which we work on
#include <pluginlib/class_list_macros.h> 
#include <panda_controllers/computed_torque.h> //library of the computed torque 

//check for the callback
#include "ros/static_assert.h"
#include <ros/console.h>

namespace panda_controllers {
  
  bool computedTorque::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle){
    
    std::string arm_id; //checking up the arm id of the robot
    if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
      return false;
    }
    
    double kp,kv;
    if(!node_handle.getParam("kp", kp) || !node_handle.getParam("kv", kv)){
      ROS_ERROR("Computed Torque: One of the parameters kp or kv couldn't be found!");
      return false;
    }
    
    Kp = kp * Eigen::MatrixXd::Identity(7,7);
    Kv = kv * Eigen::MatrixXd::Identity(7,7);
    
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names",joint_names) || joint_names.size() != 7) {
      ROS_ERROR("Computed Torque: No joint_names found!"); 
      return false;
      
    }
    
    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
      ROS_ERROR_STREAM("Computed Torque: Error getting model interface from hardware!");
      return false;
      
    }
    try {
	model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
	  ROS_ERROR_STREAM("Computed Torque: Exception getting model handle from interface: " << ex.what());
	  return false;
	  }
	  
    franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM("Computed Torque: Error getting state interface from hardware");
      return false;
      }
    try {
	 state_handle_.reset(
	 new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} 
	catch (hardware_interface::HardwareInterfaceException& ex) {
	  ROS_ERROR_STREAM("Computed Torque: Exception getting state handle from interface: " << ex.what());
	  return false;
	}
	
    hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
      ROS_ERROR_STREAM("PdController: Error getting effort joint interface from hardware!");
      return false;
      
    }
    
    for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Computed Torque: Exception getting joint handles: " << ex.what());
      return false;
      
    }
      
    }
    
    //Start command subscriber 
     
    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState>("command", 1, &computedTorque::setCommandCB, this); //it verify with the callback that the command has been received 
     
     return true;
     }
     
     void computedTorque::setCommandCB(const sensor_msgs::JointStateConstPtr &msg){
         
     }
     
     void computedTorque::starting(const ros::Time& time){
       
       //state of the robot
       franka::RobotState robot_state = state_handle_->getRobotState();
       
       //actual position of the joints
       Eigen::Map<Eigen::Matrix<double, 7, 1>> q_cur(robot_state.q.data());
       Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_cur(robot_state.dq.data());
       
       command_q = q_cur;
       
       //Let's load the mass and coriolis matrix
       std::array<double, 49> mass_array = model_handle_->getMass();
       std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
       
       Eigen::Map<Eigen::Matrix<double, 7, 7>>M(mass_array.data());
       Eigen::Map<Eigen::Matrix<double, 7, 1>>C(coriolis_array.data());
       
       //Inizializing the constants
       
       
       
       
     }
 
 
 PLUGINLIB_EXPORT_CLASS(panda_controllers::computedTorque, controller_interface::ControllerBase); 
  
}


