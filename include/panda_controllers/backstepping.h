#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <string>

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
#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"
#include "panda_controllers/link_params.h"
#include "panda_controllers/log_adaptive_cartesian.h"

#include "utils/ThunderPanda.h"
#include "utils/utils_cartesian.h"

#define     DEBUG   0      

# define    NJ 7	// number of joints
# define    PARAM 10	// number of parameters for each link

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
    
    /* Definig the timing */
    
    double dt;
    ros::Time time_now;
    
    // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
    
    Eigen::Matrix<double, NJ, 1> tau_limit;
    Eigen::Matrix<double, NJ, 1> q_min_limit;
    Eigen::Matrix<double, NJ, 1> q_max_limit;
    Eigen::Matrix<double, NJ, 1> q_dot_limit;
    
    /* Gain Matrices */
    
    Eigen::Matrix<double, 6, 6> Lambda; 
    Eigen::Matrix<double, NJ, NJ> Kd;
    Eigen::Matrix<double, NJ*PARAM, NJ*PARAM> R;
    Eigen::Matrix<double, NJ*PARAM, NJ*PARAM> Rinv;
    bool update_param_flag;

    /* Defining q_current, dot_q_current, and tau_cmd */

    Eigen::Matrix<double, NJ, 1> q_curr;
    Eigen::Matrix<double, NJ, 1> dot_q_curr;
    Eigen::Matrix<double, NJ, 1> tau_cmd;
    Eigen::Matrix<double, NJ, 1> tau_cmd_dyn;
    Eigen::Matrix<double, NJ, 1> tau_cmd_reg;

    
    Eigen::Matrix<double, NJ, 1> dot_qr;
    Eigen::Matrix<double, NJ, 1> ddot_qr;
    Eigen::Matrix<double, NJ, 1> s;

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

    Eigen::Matrix<double, NJ*PARAM, 1> param;
    Eigen::Matrix<double, NJ*PARAM, 1> dot_param;

    /* Regressor Matrix */
    
    Eigen::Matrix<double, NJ*PARAM, 1> param_dyn;
    Eigen::Matrix<double, NJ, NJ*PARAM> Yr;
    Eigen::Matrix<double, NJ, NJ> myM;
    Eigen::Matrix<double, NJ, NJ> rosM;
    Eigen::Matrix<double, NJ, NJ> myC;
    Eigen::Matrix<double, NJ, 1> rosC;
    Eigen::Matrix<double, NJ, 1> myG;
    Eigen::Matrix<double, NJ, 1> rosG;
	
    /* Object Regressor Slotine Li*/

    regrob::thunderPanda fastRegMat;

    /* Check the effort limits */
    
    Eigen::Matrix<double, NJ, 1> saturateTorqueRate (
        const Eigen::Matrix<double, NJ, 1>& tau_d_calculated,
        const Eigen::Matrix<double, NJ, 1>& tau_J_d);

    Eigen::Matrix<double, NJ, 1> tau_J_d;

    /* Import parameters */

    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command_;
    ros::Publisher pub_err_;
    ros::Publisher pub_config_;

    /* Setting Command Callback*/
    
    void setCommandCB(const desTrajEE::ConstPtr& msg);

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    /* Message */
    
    template <size_t N>
    void fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_);
    void fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_);

	panda_controllers::log_adaptive_cartesian msg_log;
    panda_controllers::point msg_config;

};

}
