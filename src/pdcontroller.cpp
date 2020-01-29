//various library on which we work onù
#include <controller_interface/controller.h> 
#include <hardware_interface/joint_command_interface.h> 
#include <pluginlib/class_list_macros.h> 

#include <panda_controllers/pdcontroller.h> //file include of the controller
#include <franka_hw/franka_state_interface.h>
#include <franka/robot_state.h>

//check for the callback
#include "ros/static_assert.h"
#include <ros/console.h>


#include <math.h>


/*#ifdef NDEBUG
#define ROS_ASSERT(cond)
#endif

#ifdef ROS_ASSERT_ENABLED
#define ROS_BREAK()
#endif

#ifdef ROS_FATALS ENABLED
#define ROS_FATALS()
#endif
*/

namespace panda_controllers 
{
    bool PdController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle)
    {
      //inizialization of the arm and setting up each joints
      
      std::string arm_id; //checking up the arm id of the robot
      if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR("PdController: Could not get parameter arm_id!");
      return false;
	 
      }
     
      double kp, kv;
      if (!node_handle.getParam("kp", kp) || !node_handle.getParam("kv", kv)) {
      ROS_ERROR("PdController: Could not get parameter kp or kv!");
      return false;
      }
      
      Kp = kp * Eigen::MatrixXd::Identity(7, 7);
      Kv = kv * Eigen::MatrixXd::Identity(7, 7);
     
    
      //Naming each joint
      
      std::vector<std::string> joint_names;
      if (!node_handle.getParam("joint_names",joint_names) || joint_names.size() != 7) {
      ROS_ERROR("PdController: No joint_names found!"); 
      return false;
      
      }
      
      //Get model interface: used to perform calculations using the dynamic model of the robot
      //in particular for modelHandle
      
      franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
      if (model_interface == nullptr) {
	ROS_ERROR_STREAM("PdController: Error getting model interface from hardware!");
	return false;
	
	}
	try {
	  model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
	  ROS_ERROR_STREAM("PdController: Exception getting model handle from interface: " << ex.what());
	  return false;
	  
	}
	
	// Get state interface: reads the full robot state, from where we can take q,qdot,tau
	
	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
	  ROS_ERROR_STREAM("PdController: Error getting state interface from hardware");
	  return false;
	  
	}
	try {
	  state_handle_.reset(
	    new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
	  ROS_ERROR_STREAM("PdController: Exception getting state handle from interface: " << ex.what());
	  return false;
	  
	}
	
	  // Getting hardware interface: command joint-level torques and read the joint states.
	  
	  hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	  if (effort_joint_interface == nullptr) {
	    ROS_ERROR_STREAM("PdController: Error getting effort joint interface from hardware!");
	    return false;
	    
	  }
	  
	   // Creating handles for each joint
	   
	   for (size_t i = 0; i < 7; ++i) {
	     try {
	      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
	       
	    } catch (const hardware_interface::HardwareInterfaceException& ex) {
	      ROS_ERROR_STREAM("PdController: Exception getting joint handles: " << ex.what());
	      return false;
	      
	    }
	     
	  }
	  
     //Start command subscriber 
     
     this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState>("command", 1, &PdController::setCommandCB, this); //it verify with the callback that the command has been received 
     
     return true;
    }
    
    void PdController::update(const ros::Time& time, const ros::Duration& period){
      
	
	franka::RobotState robot_state = state_handle_->getRobotState();
	Eigen::Map<Eigen::Matrix<double, 7, 1>> q_curr(robot_state.q.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_curr(robot_state.dq.data());
		
	//verification of the velocity vector of the joints
	if(flag){
	 
	  //Estimation of the errors
	  err = command_pos - q_curr;      
	  dot_err = command_dot_pos - dq_curr;
	  
	  /* Proportional Derivative Controller Law*/
	  tau_cmd = Kp * err + Kv * dot_err;
      
	  /* Set the command for each joint*/
      
	  for (size_t i = 0; i < 7; ++i) {
      
	    joint_handles_[i].setCommand(tau_cmd(i));
	  }
	  
	}
	else{//if the flag is false so dot_q_desired is not given
	  
	  if (q_old.data() == 0){ //if we are at the first step, where q_old (k-1 step) does not exist!
	    
// 	    for (size_t i = 0; i<7; i++) {
// 	      
// 	      periodo[i] = ones[i] * period.toSec(); //creating a vector of period
// 	      q_old[i] = ones[i] * 0.0001; //giving a q_old a small "hit"
// 	    }
	
	    dq_curr.setZero(); 
	    
	    q_old = q_curr;
	    
	    flag = false;
	    
	  }else{
	    //saving last position
	    dq_curr = (command_pos - q_old) / period.toSec();
	    
	    flag = true;
	    
	  }
	
	}  
      
    }
    
    void PdController::setCommandCB(const sensor_msgs::JointStateConstPtr &msg)//is the callback of the topic sub_command_ (up).
    {
    
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_pos((msg->position).data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_dot_pos((msg->velocity).data());
    
    do{
      if (command_pos.rows() != 7){
	ROS_FATAL("Desired position has not dimension 7! ... %d\n\tcommand_pos = %s\n",142,command_pos.rows());
	ROS_ISSUE_BREAK(); //if the vector's position is wrong, the software gives Error!
      }
    }while(0);  //loop doesn't run
    
    do{
      if (command_dot_pos.rows() != 7 || command_dot_pos.data() == 0 ){
	ROS_INFO_STREAM("Desired velocity has a wrong dimension or is not given. Velocity of the joints will be estimated.");
	flag = false;
      }
      else{
	flag = true;
      }
    }while(1);//loop continues
    
    
    
    }
  
    void PdController::starting(const ros::Time& time) {
    
    /* Getting Robot State in order to get q and q_dot */
    
    franka::RobotState robot_state = state_handle_->getRobotState();
    
    /* Remapping robot_state.q.data() and robot_state.dq.data() into Eigen Matrix Data Type */
    
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_curr(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_curr(robot_state.dq.data());
    
    /* Initialize command_pos with the current position*/
    
    command_pos = q_curr;
    
    }

    void PdController::stopping(const ros::Time& time) { }
  
    PLUGINLIB_EXPORT_CLASS(panda_controllers::PdController, controller_interface::ControllerBase); 
   
}
