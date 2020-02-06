#pragma once

#include <math.h>
#include <cmath>/*Used to getCoriolisMatrix */
#include <array>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <Eigen/Core>


#include <controller_interface/multi_interface_controller.h> //To use multiple interface template in the class backstepping definition
#include <franka/robot_state.h>
#include <controller_interface/controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>


//Ros Message
#include <sensor_msgs/JointState.h>

namespace panda_controllers
{
  

   class backstepping: public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
   hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{


public:
    
    bool init ( hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle );
    void starting ( const ros::Time& );
    void stopping ( const ros::Time& );
    void update ( const ros::Time&, const ros::Duration& period );

private:

    Eigen::MatrixXd Kd;
    Eigen::MatrixXd Delta;
    

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    //subscriber
    
    ros::Subscriber sub_command_;
    
    //callback
    
    void setCommandCB ( const sensor_msgs::JointStateConstPtr& msg );

    ros::Duration elapsed_time;
    bool flag; //check for the velocity

    /* Defining q_current, q_current_dot, q_des, and tau_cmd*/

    Eigen::Matrix<double, 7, 1> q_cur;
    Eigen::Matrix<double, 7, 1> dq_cur;
    Eigen::Matrix<double, 7, 1> tau_cmd;
    Eigen::Matrix<double, 7, 1> q_old;

    //Feedback
    
    Eigen::Matrix<double, 7, 1> error;
    Eigen::Matrix<double, 7, 1> error_dot;

    //Used for the position callback
    
    Eigen::Matrix<double, 7, 1> command_q;
    Eigen::Matrix<double, 7, 1> command_dot_q;
    Eigen::Matrix<double, 7, 1> command_dotdot_q;

    //Command for first iteration
    
    Eigen::Matrix<double, 7, 1> command_q_old;
    Eigen::Matrix<double, 7, 1> command_dot_q_old;
    Eigen::Matrix<double, 7, 1> command_dot_qref_old;

    //Used for reference velocity kinematic control q_r, q_dot_r and s
    
    Eigen::Matrix<double, 7, 1> command_dot_qref;
    Eigen::Matrix<double, 7, 1> command_dot_dot_qref;
    Eigen::Matrix<double, 7, 1> s;/* Tracking error velocity*/
    
    //Mass matrix
    
    Eigen::Matrix<double, 7, 7> M;
    
    //Coriolis matrix (array and eigen form)
    
    double Coriolis_matrix_array[49];
    Eigen::Matrix<double, 7, 7> C;
    

    //Saturation of the computedTorque
    
    Eigen::Matrix<double, 7, 1> saturateTorqueRate (
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d );

    static constexpr double kDeltaTauMax {1.0};

};

}