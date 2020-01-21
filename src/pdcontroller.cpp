//various library on which we work on
#include <controller_interface/controller.h> 
#include <hardware_interface/joint_command_interface.h> 
#include <pluginlib/class_list_macros.h> 

#include <panda_controllers/pdcontroller.h> //file include of the controller



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
    
      //Naming each joint
      std::vector<std::string> joint_names;
      if (!node_handle.getParam("joint_names",s) || joint_names.size() != 7) {
      ROS_ERROR("PdController: No joint_names found!"); joint_name
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
     sub_command_ = node_handle.subscribe<sensor_msgs::JointState>("command", 1, &PdController::setCommandCB, this); //it verify with the callback that the command has been received 
     return true;
    }
    
    void PdController::update(const ros::Time& time, const ros::Duration& period)
    {
    
      
    }
    
    void PdController::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)//is the callback of the topic sub_command_ (up).
  {
   command_ = msg->position;
  }
  
  void PdController::starting(const ros::Time& time) {
    //writing the position and velocity gain
    Kp = 100;
    Kv = 0.7;
    
    for (int i=0,i < 7,i++){
      
      std::vector<franka_hw::EffortJointInterface> q[i] = joint_handles_->getPosition[i]; //in joint handle there is the actual state of the joints of the robot
      
    }
    
    
  }
  
  void PdController::stopping(const ros::Time& time) {}
  
  PLUGINLIB_EXPORT_CLASS(panda_controllers::PdController, controller_interface::ControllerBase); 
   
}
