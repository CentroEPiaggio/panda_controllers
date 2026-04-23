#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h>

//Ros Message
#include <sensor_msgs/JointState.h>

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
  
    bool flag = false;           // flag for check of the desired command velocity
    
    /* Definig the timing */
    
    double dt;
    
    // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
    
    Eigen::Matrix<double, 7, 1> tau_limit;
    Eigen::Matrix<double, 7, 1> q_dot_limit;
    
    /* Gain Matrices */
    
    Eigen::Matrix<double, 7, 7> Kp; 
    Eigen::Matrix<double, 7, 7> Kv;
    
    Eigen::Matrix<double, 7, 7> Kp_apix; 
    Eigen::Matrix<double, 7, 7> Kv_apix;
 
    /* Defining q_current, dot_q_current, and tau_cmd */

    Eigen::Matrix<double, 7, 1> q_curr;
    Eigen::Matrix<double, 7, 1> dot_q_curr;
    Eigen::Matrix<double, 7, 1> tau_cmd;
    
    /* Error and dot error feedback */
    
    Eigen::Matrix<double, 7, 1> error;
    Eigen::Matrix<double, 7, 1> dot_error;

    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    
    Eigen::Matrix<double, 7, 1> command_q_d;           // desired command position 
    Eigen::Matrix<double, 7, 1> command_q_d_old;
    
    Eigen::Matrix<double, 7, 1> command_dot_q_d;       // desired command velocity
    Eigen::Matrix<double, 7, 1> command_dot_q_d_old;
    
    Eigen::Matrix<double, 7, 1> command_dot_dot_q_d;   // estimated desired acceleration command 

    /* Mass Matrix and Coriolis vector */
    
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> C;

    /* Check the effort limits */
    
    Eigen::Matrix<double, 7, 1> saturateTorqueRate (
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);
    
    Eigen::Matrix<double, 7, 1> tau_J_d;

    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command_;
    ros::Publisher pub_err_;
    
    /* Setting Command Callback*/
    
    void setCommandCB (const sensor_msgs::JointStateConstPtr& msg);
    
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
};

}
