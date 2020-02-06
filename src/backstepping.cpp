
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/backstepping.h> //library of the backstepping


//check for the callback
#include "ros/static_assert.h"
#include <ros/console.h>
#include <utils/get_CoriolisMatrix.h>



namespace panda_controllers
{

bool backstepping::init ( hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle )
{

    std::string arm_id; //checking up the arm id of the robot
    if ( !node_handle.getParam ( "arm_id", arm_id ) ) {
        ROS_ERROR ( "Backstepping: Could not get parameter arm_id!" );
        return false;
    }

    //Inizializing the Kd
    
    double kd, delta;
    if ( !node_handle.getParam ( "kd", kd ) || !node_handle.getParam ( "delta", delta )) {
        ROS_ERROR ( "Backstepping: Kd or delta parameter couldn't be found!" );
        return false;
    }

    Kd = kd * Eigen::MatrixXd::Identity ( 7,7 );
    
    Delta = delta * Eigen::MatrixXd::Identity ( 7,7 );

    std::vector<std::string> joint_names;
    if ( !node_handle.getParam ( "joint_names",joint_names ) || joint_names.size() != 7 ) {
        ROS_ERROR ( "Backstepping: No joint_names found!" );
        return false;

    }

    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if ( model_interface == nullptr ) {
        ROS_ERROR_STREAM ( "Backstepping: Error getting model interface from hardware!" );
        return false;

    }
    try {
        model_handle_.reset ( new franka_hw::FrankaModelHandle ( model_interface->getHandle ( arm_id + "_model" ) ) );
    } catch ( hardware_interface::HardwareInterfaceException& ex ) {
        ROS_ERROR_STREAM ( "Backstepping: Exception getting model handle from interface: " << ex.what() );
        return false;
    }

    franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if ( state_interface == nullptr ) {
        ROS_ERROR_STREAM ( "Backstepping: Error getting state interface from hardware" );
        return false;
    }
    try {
        state_handle_.reset (new franka_hw::FrankaStateHandle ( state_interface->getHandle ( arm_id + "_robot" ) ) );
    } catch ( hardware_interface::HardwareInterfaceException& ex ) {
        ROS_ERROR_STREAM ( "Backstepping: Exception getting state handle from interface: " << ex.what() );
        return false;
    }

    hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if ( effort_joint_interface == nullptr ) {
        ROS_ERROR_STREAM ( "Backstepping: Error getting effort joint interface from hardware!" );
        return false;

    }

    for ( size_t i = 0; i < 7; ++i ) {
        try {
            joint_handles_.push_back ( effort_joint_interface->getHandle ( joint_names[i] ) );

        } catch ( const hardware_interface::HardwareInterfaceException& ex ) {
            ROS_ERROR_STREAM ( "Backstepping: Exception getting joint handles: " << ex.what() );
            return false;

        }

    }

    //Start command subscriber

    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ( "command", 1, &backstepping::setCommandCB, this ); //it verify with the callback that the command has been received

    return true;
}



void backstepping::starting ( const ros::Time& time )
{

    //state of the robot
    franka::RobotState robot_state = state_handle_->getRobotState();
    
    //Loading and converting the mass matrix M
    std::array<double, 49> mass_array = model_handle_->getMass(); // Mass matrix
    
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M (mass_array.data());

    //actual position and velocity of the joints

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_cur (robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_cur (robot_state.dq.data());

    command_q = q_cur; //security inizialization
    elapsed_time = ros::Duration (0.0);

}

void backstepping::update (const ros::Time&, const ros::Duration& period)
{
    //state of the robot
    franka::RobotState robot_state = state_handle_->getRobotState();
    
    //Loading and converting the mass matrix M
    std::array<double, 49> mass_array = model_handle_->getMass(); // Mass matrix
    
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M (mass_array.data());

    //actual position of the joints
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_cur (robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_cur (robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d (robot_state.tau_J_d.data());
    
    error = command_q - q_cur;

    if ( flag ) {

        //the backstepping control law
	
	/*Calculating the Coriolis matrix (array)*/
	
	get_CoriolisMatrix(robot_state.q.data(), robot_state.dq.data(), Coriolis_matrix_array);
	
	/* Converting Coriolis matrix array in Eigen Matrix */
	
	Eigen::Map<Eigen::Matrix<double, 7, 7>> C (Coriolis_matrix_array);
	
	/* Assign command_qref_dot*/
	
	command_dot_qref = command_dot_q + Delta * error;
	
	s = command_dot_qref - dq_cur;
	
	/*Backstepping control law in the joint space*/

        tau_cmd = M * command_dot_dot_qref + C * command_dot_qref + Kd * s + error;
        
	//verification of the tau_cmd
        
	tau_cmd << saturateTorqueRate (tau_cmd , tau_J_d);
        
	//sending the torque to the joints
        
	for ( size_t i=0; i < 7; i++ ) {

            joint_handles_[i].setCommand (tau_cmd[i]);

        }


    } else {//if flag is false, so we don't have the velocity, we have to estimate it!

        if ( elapsed_time.toSec() == 0 ) { //In case of the first step, we don't have q and q_dot desired old. Briefly if we are in K = 0, we don't know the
	  //old values.
           
            command_dot_q.setZero(); //q_dot_desired = 0  
	    
	    command_q_old.setZero();
	    command_dot_q_old.setZero();
	    
	   /* Tracking reference */
	   
	    command_dot_dot_qref.setZero();
	    command_dot_qref_old.setZero();
	    
            elapsed_time += period;

            flag = false;

        }
	//Estimation of the velocities
	//k=1
	
        command_dot_q = (command_q - command_q_old) / period.toSec(); //desired velocity
	
	command_dot_qref = command_dot_q + Delta * error; //For delta we mean Lambda in the control law
	
	command_dot_dot_qref = (command_dot_qref - command_dot_qref_old) / period.toSec();

        ///saving last position and last velocity desired
        
	command_q_old = command_q;
	command_dot_qref_old = command_dot_qref;

        flag = true;

    }

}

void backstepping::stopping (const ros::Time&)
{//TO DO

}

//Check for the effort comanded if is to high
Eigen::Matrix<double, 7, 1> backstepping::saturateTorqueRate (
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d )
{
    Eigen::Matrix<double, 7, 1> tau_d_saturated {};
    for ( size_t i = 0; i < 7; i++ ) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max ( std::min ( difference, kDeltaTauMax ), -kDeltaTauMax );
    }
    return tau_d_saturated;
}


void backstepping::setCommandCB ( const sensor_msgs::JointStateConstPtr &msg )
{

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_q ( ( msg->position ).data() );

    if ( command_q.rows() != 7 ) {
        ROS_FATAL ( "Desired position has not dimension 7! ... %d\n\tcommand_q = %s\n",226,command_q.rows() );
        ROS_ISSUE_BREAK();
    }
    while ( 0 ); //blocks the compiling of the node.
    do {
        if ( command_dot_q.rows() != 7 || command_dot_q.data() == 0 ) {
            ROS_INFO_STREAM ( "Desired velocity has a wrong dimension or is not given. Velocity of the joints will be estimated." );
            flag = false;
        } else {
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_dq ( ( msg->velocity ).data() );
            flag = true;
        }
    } while ( 1 ); //loop continues


}

}
PLUGINLIB_EXPORT_CLASS ( panda_controllers::backstepping, controller_interface::ControllerBase );
