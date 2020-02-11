//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque_bis.h> //library of the computed torque 

//check for the callback
#include "ros/static_assert.h"
#include <assert.h>
#include <ros/console.h>

namespace panda_controllers
{

bool computedTorque_bis::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle)
{
    std::string arm_id; //checking up the arm id of the robot
    if (!node_handle.getParam ( "arm_id", arm_id)) {
        ROS_ERROR ("Computed Torque: Could not get parameter arm_id!");
        return false;
    }

    /* Inizializing the Kp and Kv gains */
    
    double kp,kv;
    
    if (!node_handle.getParam("kp", kp) || !node_handle.getParam ("kv", kv)) {
        ROS_ERROR ("Computed Torque: One of the parameters kp or kv couldn't be found!");
        return false;
    }

    Kp = kp * Eigen::MatrixXd::Identity (7, 7);
    Kv = kv * Eigen::MatrixXd::Identity (7, 7);

    /* Loading the Mass matrix and the Coriolis vector */
    
    std::array<double, 49> mass_array = model_handle_->getMass(); // Mass matrix
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis(); // Coriolis vector C*dq
    
    /* Mapping Mass matrix and Coriolis vector onto Eigen Matrix */
    
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> C(coriolis_array.data());

    std::vector<std::string> joint_names;
    if (!node_handle.getParam ("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR ("Computed Torque: No joint_names found!");
        return false;
    }

    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM ("Computed Torque: Error getting model interface from hardware!");
        return false;
    }
    
    try{
        model_handle_.reset (new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
    }catch(hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM ("Computed Torque: Exception getting model handle from interface: " << ex.what());
        return false;
    }

    franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM ("Computed Torque: Error getting state interface from hardware");
        return false;
    }
    
    try {
        state_handle_.reset(new franka_hw::FrankaStateHandle ( state_interface->getHandle(arm_id + "_robot")));
    }catch(hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM ("Computed Torque: Exception getting state handle from interface: " << ex.what());
        return false;
    }

    hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM ("PdController: Error getting effort joint interface from hardware!");
        return false;
    }

    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

        }catch(const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM ("Computed Torque: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    /*Start command subscriber */

    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("command", 1, &computedTorque_bis::setCommandCB, this); //it verify with the callback that the command has been received
    return true;
}

void computedTorque_bis::starting(const ros::Time& time)
{
    /* Getting Robot State in order to get q_curr and dot_q_curr */
    
    franka::RobotState robot_state = state_handle_->getRobotState();
    
    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

    /* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Eigen form  */

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_curr(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dot_q_curr(robot_state.dq.data());
    
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> C(coriolis_array.data());

    command_pos = q_curr;              // security inizialization
    
    /* Defining the NEW gains */
    
    Kp_apix = M * Kp;
    Kv_apix = M * Kp;    
}

void computedTorque_bis::update(const ros::Time&, const ros::Duration& period)
{
    franka::RobotState robot_state = state_handle_->getRobotState();
    
    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> C(coriolis_array.data());

    /* Actual position and velocity of the joints */
    
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_curr(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dot_q_curr(robot_state.dq.data());
    
    /* tau_J_d is the desired link-side joint torque sensor signals without gravity */
    
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

    if(assigned == true){

        /* The Computed Control Law */
      
        error = command_pos - q_curr;
        dot_error = command_dot_pos - dot_q_curr;
	
	/* Estimation of the command_dot_dot_pos */
	
	if (first_est == false){  
	  
	command_dot_pos_old.setZero();
	first_est = true;
	
	}
	
	command_dot_dot_pos = (command_dot_pos - command_dot_pos_old) / period.toSec();
	command_dot_pos_old = command_dot_pos; 
        
	command_pos_old = command_pos;
	
        tau_cmd = M * command_dot_dot_pos + C + Kp_apix * error + Kv_apix * dot_error;  // C->C*dq
        
	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	
        tau_cmd << saturateTorqueRate (tau_cmd , tau_J_d);
        
	/* Set the command for each joint */
	
        for (size_t i=0; i < 7; i++) {
	  
            joint_handles_[i].setCommand(tau_cmd[i]);	    
        }

    }else{ // assigned == false

        if(first_est == false) {
           
            command_dot_pos.setZero(); //q_dot_desired = 0
            command_dot_dot_pos.setZero(); //q_dot_dot_desired = 0
	    
	    command_pos_old.setZero();
	    command_dot_pos_old.setZero();

            first_est = true;
        }

        command_dot_pos = (command_pos - command_pos_old) / period.toSec(); //desired velocity

        command_dot_dot_pos = (command_dot_pos - command_dot_pos_old) / period.toSec(); //desired acceleration

        /* Saving last position and last velocity desired */
	
        command_pos_old = command_pos;
        command_dot_pos_old = command_dot_pos;
	
	tau_cmd = M * command_dot_dot_pos + C + Kp_apix * error + Kv_apix * dot_error;  // C->C*dq
        
	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	
        tau_cmd << saturateTorqueRate (tau_cmd , tau_J_d);
        
	/* Set the command for each joint */
	
        for (size_t i=0; i < 7; i++) {
	  
            joint_handles_[i].setCommand(tau_cmd[i]);	    
        }       
    }
}

void computedTorque_bis::stopping(const ros::Time&)
{  
}

/* Check for the effort commanded */

Eigen::Matrix<double, 7, 1> computedTorque_bis::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d)
{
    Eigen::Matrix<double, 7, 1> tau_d_saturated {};
    for (size_t i = 0; i < 7; i++) {
      
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min (difference, kDeltaTauMax), -kDeltaTauMax);
    }
    return tau_d_saturated;
}

void computedTorque_bis::setCommandCB(const sensor_msgs::JointStateConstPtr &msg)
{
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_pos((msg->position).data());
    
    do{
        if (command_pos.rows() != 7) {
	  
            ROS_FATAL ("Desired position has not dimension 7! ... %d\n\tcommand_pos = %s\n",215,command_pos.rows());
            ROS_ISSUE_BREAK();
	    
	}
    }while(0); //blocks the compiling of the node.
    
    if(command_pos.rows() == 7 && (msg->velocity).size() != 7 && first_est == false)
    {
      ROS_ERROR("Correct command position and invalid command velocity or not specified!Need 1st estimation!");
      assigned = false;
    }
    else if(command_pos.rows() == 7 && (msg->velocity).size() != 7 && first_est == true)
    {
      assigned = false;
    }
    else
    {
      assigned = true;
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_dot_pos((msg->velocity).data());
    }    
}

}

PLUGINLIB_EXPORT_CLASS (panda_controllers::computedTorque_bis, controller_interface::ControllerBase);
