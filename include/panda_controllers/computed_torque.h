#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <map>
#include <list>
#include <Eigen/Dense>
#include <chrono>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h>

/* thunder lib */
#include <utils/frankawrist_gen.h>
#include <utils/thunder_frankawrist.h>

//Ros Message
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define     DEBUG   0      

namespace panda_controllers
{

class ComputedTorque : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
    hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{
  
public:
  
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
    void starting(const ros::Time&);
    void stopping(const ros::Time&);
    void update(const ros::Time&, const ros::Duration& period);

private:
  
    bool flag = false; 
    bool first_msg = true;         
    
    /* Definig the timing */
    double dt;
    
    // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
    Eigen::Matrix<double, 7, 1> tau_limit;
    Eigen::Matrix<double, 7, 1> q_dot_limit;
    
    /* Diagonal matrix for damping */
    Eigen::Matrix<double, 8, 8> damping;
    
    /* Gain Matrices for Compute Torque */
    Eigen::Matrix<double, 8, 8> Kp; 
    Eigen::Matrix<double, 8, 8> Kv;
	float wrist_stiffness = 0.5;
	float wrist_theta;
	float wrist_offset;
	bool wrist_fixed;
    
    Eigen::Matrix<double, 8, 8> Kp_apix; 
    Eigen::Matrix<double, 8, 8> Kv_apix;
 
    /* Defining q_current, dot_q_current, and tau_cmd */
    Eigen::Matrix<double, 7, 1> q_curr;
    Eigen::Matrix<double, 7, 1> dot_q_curr;
    Eigen::Matrix<double, 7, 1> tau_cmd;
    // Eigen::Matrix<double, 7, 1> tau_cmd_sat;
	Eigen::Matrix<double,8,1> tau_cmd_wrist;
    
    /* Error and dot error feedback */
    Eigen::Matrix<double, 7, 1> error;
    Eigen::Matrix<double, 7, 1> dot_error;

    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    Eigen::Matrix<double, 7, 1> command_q_d;           // desired command position 
    Eigen::Matrix<double, 7, 1> command_q_d_old;
    Eigen::Matrix<double, 7, 1> command_dot_q_d;       // desired command velocity
    Eigen::Matrix<double, 7, 1> command_dot_q_d_old;
    Eigen::Matrix<double, 7, 1> command_dot_dot_q_d;   // estimated desired acceleration command 

    /* Mass Matrix, Coriolis vector and Gravity vector */
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> C;
	Eigen::Matrix<double, 7, 1> G;

    /* Check the effort limits */
    Eigen::Matrix<double, 7, 1> saturateTorqueRate (
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);
    
    Eigen::Matrix<double, 7, 1> tau_J_d;
    Eigen::Matrix<double, 7, 1> tau_J;
    Eigen::Matrix<double, 7, 1> tau_ext_filtered;

    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command;
    ros::Subscriber sub_flag;
    ros::Publisher pub_err_;
    ros::Publisher pub_torque;
    
    /* Setting Command Callback*/
    void setCommandCB (const sensor_msgs::JointStateConstPtr& msg);
    void setFlagCB (const std_msgs::BoolConstPtr& msg);
	void setTorqueCB (const std_msgs::Float64MultiArrayConstPtr& msg);
    
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    /* Franka + soft actuator */
    thunder_ns::thunder_frankawrist franka_robot;
    // Robot position, velocity and acceleration
    Eigen::Matrix<double,8,1> q_wrist;
    Eigen::Matrix<double,8,1> dq_wrist;
    // Desired positions, velocity and acceleration
    Eigen::Matrix<double,8,1> q_des_wrist;
    Eigen::Matrix<double,8,1> dq_des_wrist;
    Eigen::Matrix<double,8,1> ddq_des_wrist;
    // Position and velocity error
    Eigen::Matrix<double,8,1> error_wrist;
    Eigen::Matrix<double,8,1> dot_error_wrist;
    // Dynamic matrices and vector
    Eigen::Matrix<double, 8, 8> M_Wrist;
    Eigen::Matrix<double, 8, 8> C_Wrist;
    Eigen::Matrix<double, 8, 1> G_Wrist;
    
    // Motor angles and spring stiffness
    double theta1;
    double theta2;
    double theta_control;
    double stiffness;

    /* qbmove springs constants */
    double a1;
    double k1;
    double a2;
    double k2;

    /* ROS variables */
    ros::Subscriber sub_soft_joints;
    ros::Subscriber sub_command_wrist;
    ros::Subscriber sub_gotostart;
    ros::Publisher pub_softcommand;
    ros::Publisher pub_stiffness;
    ros::Publisher pub_thetacontrol;

    /* Setting callbacks*/
    void setJointState(const sensor_msgs::JointStateConstPtr& msg);
    void setCommandWristCB (const sensor_msgs::JointStateConstPtr& msg);
    void setGoToStartCB (const std_msgs::BoolConstPtr& msg);
};

}
