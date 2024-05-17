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
#include "panda_controllers/point.h"
#include "panda_controllers/link_params.h"
#include "panda_controllers/log_adaptive_joints.h"
#include "panda_controllers/flag.h"
#include "panda_controllers/Vec7D.h"

// #include "utils/ThunderPanda.h"
#include "utils/thunder_panda_2.h"
#include "utils/utils_param.h"

#define     DEBUG   0

#ifndef     NJ
#define     NJ 7	    // number of joints
#endif

#ifndef     PARAM
#define     PARAM 10	// number of parameters for each link
#endif

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
    ros::Time time_now;
    Eigen::Affine3d T0EE;

    // colonne matrice di massa
    Eigen::VectorXd m1;
    Eigen::VectorXd m2;
    Eigen::VectorXd m3;
    Eigen::VectorXd m4;
    Eigen::VectorXd m5;
    Eigen::VectorXd m6;
    Eigen::VectorXd m7;

    // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
    
    Eigen::Matrix<double, 7, 1> tau_limit;
    Eigen::Matrix<double, 7, 1> q_dot_limit;
    
    /* Gain Matrices */
    
    Eigen::Matrix<double, 7, 7> Kp; 
    Eigen::Matrix<double, 7, 7> Kv;
    
    Eigen::Matrix<double, 7, 7> Kp_apix; 
    Eigen::Matrix<double, 7, 7> Kv_apix;

    /* Gain for parameters */

    Eigen::Matrix<double, NJ*PARAM, NJ*PARAM> Rinv;
    Eigen::Matrix<double, 14 ,14> A;
    Eigen::Matrix<double, 14 ,7> B;
    Eigen::Matrix<double, 14, 14> P;
    Eigen::Matrix<double, 14, 14> Q;
    bool update_param_flag;
 
    /* Defining q_current, dot_q_current, and tau_cmd */

    Eigen::Matrix<double, 7, 1> q_curr;
    Eigen::Matrix<double, 7, 1> dot_q_curr;
    Eigen::Matrix<double, 7, 1> dot_q_curr_old;
    Eigen::Matrix<double, 7, 1> ddot_q_curr;
    Eigen::Matrix<double, 7, 1> tau_cmd;
    Eigen::Matrix<double, 7, 1> ddot_q_curr_old;

    /*Variabili filtro Media mobile*/
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_dq; // Array dinamico 7D
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_ddq;
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_tau;
    const int WIN_LEN = 6;
    
    /* Error and dot error feedback */
    
    Eigen::Matrix<double, 7, 1> error;
    Eigen::Matrix<double, 7, 1> dot_error;
    Eigen::Matrix<double, 14, 1> x;


    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    
    Eigen::Matrix<double, 7, 1> command_q_d;           // desired command position 
    Eigen::Matrix<double, 7, 1> command_q_d_old;
    
    Eigen::Matrix<double, 7, 1> command_dot_q_d;       // desired command velocity
    Eigen::Matrix<double, 7, 1> command_dot_q_d_old;
    
    Eigen::Matrix<double, 7, 1> command_dot_dot_q_d;   // estimated desired acceleration command 

    /* Mass Matrix and Coriolis vector */
    
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> C;
    Eigen::Matrix<double, 7, 1> G;
    Eigen::Matrix<double, 7, 7> Mest;
    Eigen::Matrix<double, 7, 7> Cest;
    Eigen::Matrix<double, 7, 1> Gest;
    
    /* Parameter vector */

    Eigen::Matrix<double, NJ*PARAM, 1> param;    // usfull for calculate an estimate of M, C, G 
    Eigen::Matrix<double, NJ*PARAM, 1> dot_param;
    Eigen::Matrix<double, NJ*PARAM, 1> param_dyn; 

    /* Regressor Matrix */
    
    Eigen::Matrix<double, NJ, NJ*PARAM> Y;

    /* Object Regressor Slotine Li*/

    thunder_ns::thunder_panda_2 fastRegMat;

    /*Filter function*/
    void aggiungiDato(std::vector<Eigen::Matrix<double, 7, 1>>& buffer_, const Eigen::Matrix<double, 7, 1>& dato_, int lunghezza_finestra_);
    Eigen::Matrix<double, 7, 1> calcolaMedia(const std::vector<Eigen::Matrix<double, 7, 1>>& buffer_);
    
    /* Check the effort limits */
    
    Eigen::Matrix<double, 7, 1> saturateTorqueRate (
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);
    // Eigen::Matrix<double, 14, 14> solveContinuousLyapunov(const Eigen::Matrix<double,14,14>& A, const Eigen::Matrix<double,14,14>& Q);


    Eigen::Matrix<double, 7, 1> tau_J_d;

    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_flag_update_;
    ros::Publisher pub_err_;
    ros::Publisher pub_config_;
    ros::Publisher pub_deb_;
    
    /* Setting Command Callback*/
    
    void setCommandCB (const sensor_msgs::JointStateConstPtr& msg);
    
    /*Setting Flag Callback*/
    void setFlagUpdate(const flag::ConstPtr& msg);

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;


    /* Message */
    
    template <size_t N>
    void fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_);
    void fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_);

	panda_controllers::log_adaptive_joints msg_log;
    panda_controllers::point msg_config;
    panda_controllers::Vec7D msg_deb;
};

}
