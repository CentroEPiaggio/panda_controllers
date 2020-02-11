//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/pdcontroller.h>

//check for the callback
#include "ros/static_assert.h"
#include <assert.h>
#include <ros/console.h>

namespace panda_controllers
{
  
bool PdController::init (hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle)
{
    //inizialization of the arm and setting up each joints

    std::string arm_id; //checking up the arm id of the robot
    if (!node_handle.getParam("arm_id", arm_id)){
        ROS_ERROR("PdController: Could not get parameter arm_id!");
        return false;
    }

    double kp, kv;
    
    if (!node_handle.getParam("kp", kp) || !node_handle.getParam("kv", kv)){
        ROS_ERROR("PdController: Could not get parameter kp or kv!");
        return false;
    }

    Kp = kp * Eigen::MatrixXd::Identity (7, 7);
    Kv = kv * Eigen::MatrixXd::Identity (7, 7);

    //Naming each joint

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR ("PdController: No joint_names found!");
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
    }catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("PdController: Exception getting model handle from interface: " << ex.what());
        return false;
    }

    // Get state interface: reads the full robot state, from where we can take q, qdot, tau

    franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("PdController: Error getting state interface from hardware");
        return false;
    }
    
    try {
        state_handle_.reset (new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
    }catch (hardware_interface::HardwareInterfaceException& ex) {
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
            joint_handles_.push_back( effort_joint_interface->getHandle(joint_names[i]));

        } catch (const hardware_interface::HardwareInterfaceException& ex ) {
            ROS_ERROR_STREAM("PdController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    /* Start command subscriber */

    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState>("command", 1, &PdController::setCommandCB, this); //it verify with the callback that the command has been received
    return true;
}

void PdController::starting(const ros::Time& time)
{
    /* Getting Robot State in order to get q_curr and dot_q_curr */

    franka::RobotState robot_state = state_handle_->getRobotState();

    /* Mapping actual joints position and actual joints velocity onto Eigen form */

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_curr(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dot_q_curr(robot_state.dq.data());
  
    command_q_d = q_curr;              // security inizialization
    elapsed_time = ros::Duration(0.0); // first estimation
}

void PdController::update(const ros::Time& time, const ros::Duration& period)
{
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_curr(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dot_q_curr(robot_state.dq.data());
    
    /* tau_J_d is the desired link-side joint torque sensor signals without gravity */
    
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());  

    // verification of the velocity vector of the joints
    
    if (flag) {

        // Estimation of the errors
      
        err = command_q_d - q_curr;
        dot_err = command_dot_q_d - dot_q_curr;
	
	/* DUBBIO */
	
	/* Se siamo nella situazione in cui ho assegnato un comando di posizione, e di velocità,
	 al termine di esso svuoto il vettore del comando di velocità, quindi assegno solamente la 
	 il comando di posizione:
	 
	 1) Devo evitare che entri nel ciclo if(elapsed_time.toSec() == 0), quindi magari
	 vado ad incrementare la variabile
	 
	 2) devo salvarmi l'ultima command_q_d, che nella stima sarà la command_q_d_old*/
        
	command_q_d_old = command_q_d;
	elapsed_time += period;
	
        /* Proportional Derivative Controller Law */
        
	tau_cmd = Kp * err + Kv * dot_err;
	
	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */

        tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

        /* Set the command for each joint */

        for (size_t i = 0; i < 7; ++i) {

            joint_handles_[i].setCommand(tau_cmd(i));
	    
        }

    }else{ //if the flag is false so dot_q_desired is not given

        if (elapsed_time.toSec() == 0) { //if we are at the first step, where q_old (k-1 step) does not exist!

            command_dot_q_d.setZero();
	    
 	    command_q_d_old.setZero();

            flag = false;

            elapsed_time += period;

        }else{

            // estimation of command_dot_q_d
            
	    command_dot_q_d = (command_q_d - command_q_d_old) / period.toSec();
            
	    // saving last position
            
	    command_q_d_old = command_q_d;

            flag = true;
        }
    }
}

void PdController::stopping(const ros::Time& time) {}

/* Check for the effort commanded */

Eigen::Matrix<double, 7, 1> PdController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d)
{
    Eigen::Matrix<double, 7, 1> tau_d_saturated {};
    for (size_t i = 0; i < 7; i++) {
      
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max (std::min(difference, kDeltaTauMax), -kDeltaTauMax);
    }
    return tau_d_saturated;
}

void PdController::setCommandCB(const sensor_msgs::JointStateConstPtr &msg) //is the callback of the topic sub_command_ (up).
{
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_q_d((msg->position).data());
    
    do{
        if (command_q_d.rows() != 7) {
            ROS_FATAL ("Desired position has not dimension 7! ... %d\n\tcommand_q_d = %s\n",202,command_q_d.rows());
            ROS_ISSUE_BREAK(); //if the vector's position is wrong, the software gives Error!
        }
    }while(0); //loop doesn't run

    do{
        if ((msg->velocity).size() != 7 || (msg->velocity).empty()) {
	  
            ROS_INFO_STREAM ("Desired velocity has a wrong dimension or is not given. Velocity of the joints will be estimated.");
            flag = false;
	    
        }else{
	  
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_dot_q_d((msg->velocity).data());
            flag = true;
	    
        }
    }while(1); // loop continues(means while(1!=0))
}

}

PLUGINLIB_EXPORT_CLASS (panda_controllers::PdController, controller_interface::ControllerBase);
