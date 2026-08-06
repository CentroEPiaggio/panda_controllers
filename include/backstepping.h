#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>

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
#include <geometry_msgs/PoseStamped.h>
#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"
#include "panda_controllers/link_params.h"
#include "panda_controllers/log_adaptive_cartesian.h"
#include "panda_controllers/flag.h"
#include <franka_msgs/SetEEFrame.h>

#include "thunder_franka.h"
#include "utils_cartesian.h"

#define     DEBUG   0      

#define    NJ      9       // number of joints
#define    ndof    7       // number of degrees of freedom

#ifndef     PARAM
# define    PARAM 10	// number of parameters for each link
#endif

namespace panda_controllers
{

class Backstepping : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
    hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{
  
public:
  
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
    void starting(const ros::Time&);
    void stopping(const ros::Time&);
    void update(const ros::Time&, const ros::Duration& period);

private:
  
    bool flag = false;           // flag for check of the desired command velocity
	bool logging;
    const double tol_s = 0.01;
    const double UB_s = 1;
    // const int NJ = 9;
    // const int ndof = 7;
    
    /* Definig the timing */
    
    double dt;
    ros::Time time_now;

    /* Robot state handle */

    franka::RobotState robot_state;
    
    /* Franka ROS matrices */

    Eigen::Matrix<double, 6, ndof> Jee;
	Eigen::Matrix4d T0EE;
    Eigen::Matrix<double, ndof, 1> franka_G;

    // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
    
    Eigen::Matrix<double, ndof, 1> tau_limit;
    Eigen::Matrix<double, ndof, 1> q_min_limit;
    Eigen::Matrix<double, ndof, 1> q_max_limit;
    Eigen::Matrix<double, ndof, 1> q_dot_limit;
    
    /* Gain Matrices */
    
    Eigen::Matrix<double, 6, 6> Lambda; 
    Eigen::Matrix<double, ndof, ndof> Kd;
    //Eigen::Matrix<double, NJ*PARAM, NJ*PARAM> R;
    Eigen::Matrix<double, NJ*PARAM, NJ*PARAM> Rinv;
    bool update_kin_flag = false;
	bool update_dyn_flag = false;
    bool UB_s_flag = false;

    /* Defining q_current, dot_q_current, s and tau_cmd */

    Eigen::Matrix<double, ndof, 1> q_curr;
    Eigen::Matrix<double, ndof, 1> dot_q_curr;
    Eigen::Matrix<double, ndof, 1> dot_qr;
    Eigen::Matrix<double, ndof, 1> ddot_qr;
    Eigen::Matrix<double, ndof, 1> s;
    Eigen::Matrix<double, ndof, 1> tau_cmd;
    Eigen::Matrix<double, ndof, 1> tau_tilde;
    
    /* Error and dot error feedback */
    
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> dot_error;

    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    
    Eigen::Matrix<double, 3, 1> ee_pos_cmd;             // desired command position 
    Eigen::Matrix<double, 3, 1> ee_vel_cmd;             // desired command velocity 
    Eigen::Matrix<double, 3, 1> ee_acc_cmd;             // desired command acceleration 
    
    //Eigen::Matrix<double, 3, 1> ee_ang_cmd;             // desired command position
    Eigen::Matrix<double, 3, 3> ee_rot_cmd;             // desired command position
    Eigen::Matrix<double, 3, 1> ee_ang_vel_cmd;         // desired command velocity 
    Eigen::Matrix<double, 3, 1> ee_ang_acc_cmd;         // desired command acceleration 

    /* Parameter vector */

    Eigen::Matrix<double, 90, 1> param_REG;
	Eigen::Matrix<double,6,1> ee_tr;
    Eigen::Matrix<double, 90, 1> param_init;
    Eigen::Matrix<double, 90, 1> dot_param;

    /* Regressor Matrix */
    
    Eigen::Matrix<double, 7, 90> Yr;
	
	/* Pseudo-inverse of jacobian and its derivative matrices */
	
	Eigen::Matrix<double,ndof,6> J_pinv;
	Eigen::Matrix<double,6,ndof> J_dot;

    /* Object Regressor Slotine Li*/

    thunder_franka franka;

    /* Check the effort limits */
    
    Eigen::Matrix<double, ndof, 1> saturateTorqueRate (
        const Eigen::Matrix<double, ndof, 1>& tau_d_calculated,
        const Eigen::Matrix<double, ndof, 1>& tau_J_d);

    Eigen::Matrix<double, ndof, 1> tau_J_d;

    /* Import parameters */

    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_flag_update_;
    ros::Publisher pub_log;
    ros::Publisher pub_config_;
    ros::Publisher pub_franka_pose;

    /* Setting Command Callback*/
    
    void setCommandCB(const desTrajEE::ConstPtr& msg);

    /*Setting Flag Callback*/
    void setFlagUpdate(const flag::ConstPtr& msg);

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    /* Message */
    
    template <size_t N>
    void fillMsg(boost::array<float, N>& msg_, const Eigen::MatrixXd& data_);
    // void fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_);

	panda_controllers::log_adaptive_cartesian msg_log;
    panda_controllers::point msg_config;


};

}
