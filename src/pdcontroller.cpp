//various library on which we work on
#include <controller_interface/controller.h> 
#include <hardware_interface/joint_command_interface.h> 
#include <pluginlib/class_list_macros.h> 

#include <panda_controllers/pdcontroller.h> //file include of the controller
#include <franka_hw/franka_state_interface.h>
#include <franka/robot_state.h>


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
      
      /* Proportional Derivative Controller Law*/
      
      tau_cmd = Kp * (command_dot_pos - dq_curr) + Kv *(command_dot_pos - dq_curr);
      
      /* Set command for each joint*/
      
      for (size_t i = 0; i < 7; ++i) {
      
	joint_handles_[i].setCommand(tau_cmd(i));
      }
      
    }
    
    void PdController::setCommandCB(const sensor_msgs::JointStateConstPtr &msg)//is the callback of the topic sub_command_ (up).
    {
    
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_pos((msg->position).data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_dot_pos((msg->velocity).data());
    
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
