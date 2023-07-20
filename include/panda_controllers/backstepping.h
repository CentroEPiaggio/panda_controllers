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
#include "panda_controllers/desTrajEE.h"
#include "panda_controllers/log_backstepping.h"

#include "utils/ThunderPanda.h"

#define     DEBUG   0      

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
    Eigen::Matrix<double, 7, 1> ddot_q_curr;
    Eigen::Matrix<double, 7, 1> tau_cmd;
    
    /* Error and dot error feedback */
    
    Eigen::Matrix<double, 7, 1> error;
    Eigen::Matrix<double, 7, 1> dot_error;

    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    
    Eigen::Matrix<double, 3, 1> ee_pos_cmd;           // desired command position 
    Eigen::Matrix<double, 3, 1> ee_vel_cmd;           // desired command velocity 
    Eigen::Matrix<double, 3, 1> ee_acc_cmd;           // desired command acceleration 

    /* Parameter vector */

    Eigen::Matrix<double, 70, 1> param;

    /* Mass Matrix and Coriolis vector */
    
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> C;
    Eigen::Matrix<double, 7, 1> G;

    Eigen::Matrix<double, 7, 70>Yr;
	
    /* Object Regressor Slotine Li*/

    //regrob::SLregressor regressor;
    regrob::thunderPanda fastRegMat;


    /* Check the effort limits */
    
    Eigen::Matrix<double, 7, 1> saturateTorqueRate (
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);

    Eigen::Matrix<double, 7, 1> tau_J_d;

    /* Import parameters */

    Eigen::Matrix<double, 70, 1> importCSV(const std::string& filename);

    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command_;
    ros::Publisher pub_err_;
    
    /* Setting Command Callback*/
    
    void setCommandCB(const desTrajEE::ConstPtr& msg);

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    template <size_t N>
    void fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_);

};

}
