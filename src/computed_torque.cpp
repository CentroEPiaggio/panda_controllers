//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque.h> //library of the computed torque 

//check for the callback
#include "ros/static_assert.h"
#include <ros/console.h>

namespace panda_controllers
{

bool computedTorque::init ( hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle )
{

    std::string arm_id; //checking up the arm id of the robot
    if ( !node_handle.getParam ( "arm_id", arm_id ) ) {
        ROS_ERROR ( "Computed Torque: Could not get parameter arm_id!" );
        return false;
    }

    //Inizializing the Kp and Kv gains
    double kp,kv;
    if ( !node_handle.getParam ( "kp", kp ) || !node_handle.getParam ( "kv", kv ) ) {
        ROS_ERROR ( "Computed Torque: One of the parameters kp or kv couldn't be found!" );
        return false;
    }

    Kp = kp * Eigen::MatrixXd::Identity ( 7,7 );
    Kv = kv * Eigen::MatrixXd::Identity ( 7,7 );

    //Loading the mass and coriolis matrix
    std::array<double, 49> mass_array = model_handle_->getMass(); // Mass matrix
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis(); // C*dq

    Eigen::Map<Eigen::Matrix<double, 7, 7>>M ( mass_array.data() );
    Eigen::Map<Eigen::Matrix<double, 7, 1>>C ( coriolis_array.data() );

    std::vector<std::string> joint_names;
    if ( !node_handle.getParam ( "joint_names",joint_names ) || joint_names.size() != 7 ) {
        ROS_ERROR ( "Computed Torque: No joint_names found!" );
        return false;

    }

    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if ( model_interface == nullptr ) {
        ROS_ERROR_STREAM ( "Computed Torque: Error getting model interface from hardware!" );
        return false;

    }
    try {
        model_handle_.reset ( new franka_hw::FrankaModelHandle ( model_interface->getHandle ( arm_id + "_model" ) ) );
    } catch ( hardware_interface::HardwareInterfaceException& ex ) {
        ROS_ERROR_STREAM ( "Computed Torque: Exception getting model handle from interface: " << ex.what() );
        return false;
    }

    franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if ( state_interface == nullptr ) {
        ROS_ERROR_STREAM ( "Computed Torque: Error getting state interface from hardware" );
        return false;
    }
    try {
        state_handle_.reset (
            new franka_hw::FrankaStateHandle ( state_interface->getHandle ( arm_id + "_robot" ) ) );
    } catch ( hardware_interface::HardwareInterfaceException& ex ) {
        ROS_ERROR_STREAM ( "Computed Torque: Exception getting state handle from interface: " << ex.what() );
        return false;
    }

    hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if ( effort_joint_interface == nullptr ) {
        ROS_ERROR_STREAM ( "PdController: Error getting effort joint interface from hardware!" );
        return false;

    }

    for ( size_t i = 0; i < 7; ++i ) {
        try {
            joint_handles_.push_back ( effort_joint_interface->getHandle ( joint_names[i] ) );

        } catch ( const hardware_interface::HardwareInterfaceException& ex ) {
            ROS_ERROR_STREAM ( "Computed Torque: Exception getting joint handles: " << ex.what() );
            return false;

        }

    }

    //Start command subscriber

    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ( "command", 1, &computedTorque::setCommandCB, this ); //it verify with the callback that the command has been received

    return true;
}

void computedTorque::setCommandCB ( const sensor_msgs::JointStateConstPtr &msg )
{

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_q ( ( msg->position ).data() );

    if ( command_q.rows() != 7 ) {
        ROS_FATAL ( "Desired position has not dimension 7! ... %d\n\tcommand_q = %s\n",96,command_q.rows() );
        ROS_ISSUE_BREAK();
    }
    while ( 0 ); //blocks the compiling of the node.
    do {
        if ( command_dq.rows() != 7 || command_dq.data() == 0 ) {
            ROS_INFO_STREAM ( "Desired velocity has a wrong dimension or is not given. Velocity of the joints will be estimated." );
            flag = false;
        } else {
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> command_dq ( ( msg->velocity ).data() );
            flag = true;
        }
    } while ( 1 ); //loop continues


}

void computedTorque::starting ( const ros::Time& time )
{

    //state of the robot
    franka::RobotState robot_state = state_handle_->getRobotState();

    //actual position and velocity of the joints

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_cur ( robot_state.q.data() );
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_cur ( robot_state.dq.data() );

    command_q = q_cur; //security inizialization

    //Defining the NEW gains
    Kp_apix = M * Kp;
    Kv_apix = M * Kp;

    elapsed_time = ros::Duration ( 0.0 );


}

void computedTorque::update ( const ros::Time&, const ros::Duration& period )
{
    //state of the robot
    franka::RobotState robot_state = state_handle_->getRobotState();

    //actual position of the joints
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_cur ( robot_state.q.data() );
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_cur ( robot_state.dq.data() );
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d ( robot_state.tau_J_d.data() );

    if ( flag ) {

        //the computed torque law
        error = command_q - q_cur;
        error_dot = command_dq - dq_cur;

        tau_cmd = M * command_dotdot_q + C + Kp_apix * error + Kv_apix * error_dot;
        
	//verification of the tau_cmd
        tau_cmd << saturateTorqueRate ( tau_cmd , tau_J_d );
        
	//sending the torque to the joints
        for ( size_t i=0; i<7; i++ ) {

            joint_handles_[i].setCommand ( tau_cmd[i] );

        }


    } else {//if flag is false, so we don't have the velocity, we have to estimate it.

        if ( elapsed_time.toSec() == 0 ) { //In case of the first step, we don't have q and q_dot desired old.
           
            command_dq.setZero(); //q_dot_desired = 0
            command_dotdot_q.setZero(); //q_dot_dot_desired = 0
	    
	    command_q_old.setZero();
	    command_dq_old.setZero();

            elapsed_time += period;

            flag = false;

        }

        command_dq = ( command_q - command_q_old ) / period.toSec(); //desired velocity

        command_dotdot_q = ( command_dq - command_dq_old ) / period.toSec(); //desired acceleration

        ///saving last position and last velocity desired
        command_q_old = command_q;
        command_dq_old = command_dq;

        flag = true;


    }


}

//Check for the effort comanded if is to high
Eigen::Matrix<double, 7, 1> computedTorque::saturateTorqueRate (
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


void computedTorque::stopping ( const ros::Time& )
{

}



PLUGINLIB_EXPORT_CLASS ( panda_controllers::computedTorque, controller_interface::ControllerBase );

}


