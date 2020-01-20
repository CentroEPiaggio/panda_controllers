//various library on which we work on
#include <controller_interface/controller.h> 
#include <hardware_interface/joint_command_interface.h> 
#include <pluginlib/class_list_macros.h> 

#include <panda_controllers/pdcontroller.h> //file include of the controller



namespace panda_controllers 
{
    bool PdController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
      //inizialization of the joints
     std::string my_joint;
     if (!n.getParam("joint_names", my_joint))
     {
       ROS_ERROR("Could not find joint name!");
       return false;       
     }
     
     joint_ = hw->getHandle(my_joint); //Once the joint is found, we are going to listen that specific joint
     command_ = joint_.getPosition(); //set the current joint goal to the current joint getPosition 
     
     //Start command subscriber 
     sub_command_ = n.subscribe<sensor_msgs::JointState>("command", 1, &PdController::setCommandCB, this); //it verify with the callback that the command has been received 
     return true;
    }
  
  void PdController::update(const ros::Time& time, const ros::Duration& period)
  {
    error = command_ - joint_.getPosition(); //the error is between the command received (command_) and the actual position of the joint (joint_.getPosition)
    
    dt = 2;
    dot_error = (error - old_error)/dt; 
    
    commanded_effort = Kp*error + Kv*dot_error;//application of the pd controller
    
    joint_.setCommand(commanded_effort); //set the torque to the joints
    
    old_error = error;
  }
  
  void PdController::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)//is the callback of the topic sub_command_ (up).
  {
   command_ = msg->position;
  }
  

  void PdController::starting(const ros::Time& time) {
    
    for(i=0,i < 6,i++){
      
      Kp(i) = 100;
      Kv(i) = 0.7;
      
    }
    
  }
  
  void PdController::stopping(const ros::Time& time) {}
  
  PLUGINLIB_EXPORT_CLASS(panda_controllers::PdController, controller_interface::ControllerBase);
   
}
