#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h>

//Ros Message
#include <sensor_msgs/JointState.h>

namespace panda_controllers
{

class computedTorque_bis : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
    hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{


public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
    void starting(const ros::Time&);
    void stopping(const ros::Time&);
    void update(const ros::Time&, const ros::Duration& period);

private:
    bool assigned = false;
    bool first_est = false;
    
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
    
    Eigen::Matrix<double, 7, 1> command_pos;           // desired command position
    Eigen::Matrix<double, 7, 1> command_pos_old;       // Extra definition for the estimation of the command_dot_pos 
    
    Eigen::Matrix<double, 7, 1> command_dot_pos;       // desired command velocity
    Eigen::Matrix<double, 7, 1> command_dot_pos_old;   // Extra definition for the estimation of the command_dotdot_pos 
    
    Eigen::Matrix<double, 7, 1> command_dot_dot_pos;   // estimated desired acceleration command 

    /* Mass Matrix and Coriolis vector */
    
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> C;

    /* Check the effort limits */
    
    Eigen::Matrix<double, 7, 1> saturateTorqueRate (
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);

    static constexpr double kDeltaTauMax {1.0};
    
    /* For listening to the command topic */
   
    ros::Subscriber sub_command_;
    
    /* Setting Command Callback*/
    
    void setCommandCB (const sensor_msgs::JointStateConstPtr& msg);
    
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
};

}
