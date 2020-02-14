#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include <controller_interface/multi_interface_controller.h> /*This particular controller implementation allows to claim resources from one up to four different hardware interfaces. 
MultiInterfaceController <first interface, second interface, ....> */

#include <franka_hw/franka_model_interface.h> /* Handle to perform calculations using the dynamic model of a robot: franka_hw::FrankaModelHandle*/
#include <franka_hw/franka_state_interface.h> /* Handle to read the complete state of a robot: franka_hw::FrankaStateHandle */

#include <hardware_interface/robot_hw.h> /* This class provides a standardized interface to a set of robot hardware interfaces to the controller manager. It performs resource conflict checking for a given set of controllers and maintains a map of hardware interfaces. 
It is meant to be used as a base class for abstracting custom robot hardware: hardware_interface::RobotHW (see function get())*/
#include <hardware_interface/joint_command_interface.h> /* A handle used to read and command a single joint: hardware_interface::JointHandle */

#include <ros/node_handle.h> /* roscpp's interface for creating subscribers, publishers, etc. */
#include <ros/time.h> 

#include <franka/robot_state.h> /* Describe the robot state: q_curr, dot_q_curr...*/

//ROS message
#include <sensor_msgs/JointState.h>

namespace panda_controllers
{

class PdController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
    hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{

public:
  
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
    void starting(const ros::Time&);
    void stopping(const ros::Time&);
    void update(const ros::Time&, const ros::Duration& period);

private:

    bool flag = false;           // flag true if desired velocity is given
    
    /* Defining Position and Velocity Gains */
    
    Eigen::Matrix<double, 7, 7> Kp; 
    Eigen::Matrix<double, 7, 7> Kv;

    /* Defining q_current, dot_q_current, and tau_cmd */

    Eigen::Matrix<double, 7, 1> q_curr;
    Eigen::Matrix<double, 7, 1> dot_q_curr;
    Eigen::Matrix<double, 7, 1> tau_cmd;
    
    /* Error and dot error feedback */
    
    Eigen::Matrix<double, 7, 1> err;
    Eigen::Matrix<double, 7, 1> dot_err;
    
    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
       
    Eigen::Matrix<double, 7, 1> command_q_d;      // desired command position
    Eigen::Matrix<double, 7, 1> command_q_d_old;  // Extra definition for the estimation of the command_dot_pos 
    
    Eigen::Matrix<double, 7, 1> command_dot_q_d;  // desired command velocity
    
    /* Check the effort limits */
     
    Eigen::Matrix<double, 7, 1> saturateTorqueRate (
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);
    
    Eigen::Matrix<double, 7, 1> tau_J_d;

    static constexpr double kDeltaTauMax {1.0};
    
    /* For listening to the command topic */
    
    ros::Subscriber sub_command_;
    
    /* Setting Command Callback*/

    void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);
    
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; //used for the state of the joints and command the torque
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    /*unique_ptr----->These objects have the ability of taking ownership of a pointer: once they take ownership
     *they manage the pointed object by becoming responsible for its deletion at some point.*/
};

}
