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
#include "panda_controllers/desTrajEE.h"
#include "panda_controllers/link_params.h"
#include "panda_controllers/log_adaptive_cartesian.h"
#include "panda_controllers/flag.h"

#include "utils/ThunderPanda.h"
#include "utils/utils_cartesian.h"
#include "utils/utils_param.h"

#define     DEBUG   0      

#ifndef     NJ
# define    NJ 7	// number of joints
#endif

#ifndef     PARAM
# define    PARAM 10	// number of parameters for each link
#endif

#ifndef     FRICTION
#define     FRICTION 2	// number of friction parameters for each link
#endif

namespace panda_controllers
{

class CTModOS : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
    hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{
  
public:
  
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
    void starting(const ros::Time&);
    void stopping(const ros::Time&);
    void update(const ros::Time&, const ros::Duration& period);

private:
  
    bool flag = true;           // flag for check of the desired command velocity
    
    /* Definig the timing */
    
    double dt;
    ros::Time time_now;
    

    /* Robot state handle */

    franka::RobotState robot_state;
    
    /* Franka ROS matrices */

    Eigen::Matrix<double, 6, NJ> jacobian;
	Eigen::Affine3d T0EE;
    

    // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
    
    Eigen::Matrix<double, NJ, 1> tau_limit;
    Eigen::Matrix<double, NJ, 1> q_min_limit;
    Eigen::Matrix<double, NJ, 1> q_max_limit;
    Eigen::Matrix<double, NJ, 1> q_dot_limit;
    
    /* Gain Matrices */
    Eigen::Matrix<double, 6, 6> Lambda; 
    Eigen::Matrix<double, 6, 6> Kp; 
    Eigen::Matrix<double, 7, 7> Kv;
    Eigen::Matrix<double, 6, 6> Kp_xi; 
    Eigen::Matrix<double, 7, 7> Kv_xi;
   
     /* Gain for parameters */

    Eigen::Matrix<double, NJ*PARAM, NJ*PARAM> Rinv;
    Eigen::Matrix<double, NJ*(FRICTION), NJ*(FRICTION)> Rinv_fric;
    bool update_param_flag;

    /* Defining q_current, dot_q_current, s and tau_cmd */

    Eigen::Matrix<double, NJ, 1> q_curr;
    Eigen::Matrix<double, NJ, 1> q_c; // vettore giunti meta corsa
    Eigen::Matrix<double, NJ, 1> dot_q_curr;
    Eigen::Matrix<double, NJ, 1> dot_q_curr_old;
    Eigen::Matrix<double, NJ, 1> ddot_q_curr;
    Eigen::Matrix<double, NJ, 1> dot_qr;
    Eigen::Matrix<double, NJ, 1> ddot_qr;
    Eigen::Matrix<double, NJ, 1> dot_error_q;
    
    Eigen::Matrix<double, NJ, 1> err_param;
    Eigen::Matrix<double, 7, 1> err_param_frict;

    Eigen::Matrix<double, NJ, 1> tau_cmd;
    Eigen::Matrix<double, NJ, 1> tau_J;
    Eigen::Matrix<double, 6, 1> F_cmd; // Forza commandata agente sull'EE
    Eigen::Matrix<double, 6, 1> vel_cur;
    Eigen::Matrix<double, NJ, 1> tau_tilde;
    
    /* Error and dot error feedback */
    
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> dot_error;
    Eigen::Matrix<double, 6, 1> ddot_error;

    Eigen::Vector3d ee_position, ee_velocity, ee_acceleration;
    Eigen::Vector3d ee_omega;

    Eigen::Matrix<double,3,3> ee_rot;
    Eigen::Matrix<double,3,3> Rs_tilde;
    Eigen::Matrix<double,6,6> L, L_dot;

    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    
    Eigen::Matrix<double, 3, 1> ee_pos_cmd;             // desired command position 
    Eigen::Matrix<double, 3, 1> ee_vel_cmd;             // desired command velocity 
    Eigen::Matrix<double, 3, 1> ee_acc_cmd;             // desired command acceleration 
    
    //Eigen::Matrix<double, 3, 1> ee_ang_cmd;             // desired command position
    Eigen::Matrix<double, 3, 3> ee_rot_cmd;             // desired command position
    Eigen::Matrix<double, 3, 1> ee_ang_vel_cmd;         // desired command velocity 
    Eigen::Matrix<double, 3, 1> ee_ang_acc_cmd;         // desired command acceleration 

    std::vector<Eigen::Matrix<double, 7, 1>> buffer_dq; // Array dinamico 7D
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_ddq;
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_tau;
    const int WIN_LEN = 6;

    /* Parameter vector */

    Eigen::Matrix<double, NJ*PARAM, 1> param;
    Eigen::Matrix<double, NJ*PARAM, 1> param_init;
    Eigen::Matrix<double, NJ*PARAM, 1> dot_param;
    Eigen::Matrix<double, NJ*PARAM, 1> param_dyn;
    Eigen::Matrix<double, NJ*(FRICTION), 1> param_frict;
    Eigen::Matrix<double, NJ*(FRICTION), 1> dot_param_frict;
    Eigen::Matrix<double, NJ*(PARAM+FRICTION), 1> param_tot;

    /* Mass Matrix and Coriolis vector */
        
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> C; 
    Eigen::Matrix<double, 7, 1> G;

    /* Mass Matrix and Coriolis vector with regressor calculation*/
    Eigen::Matrix<double, 7, 7> Mest;
    Eigen::Matrix<double, 7, 7> Cest;
    Eigen::Matrix<double, 7, 7> I7;
    Eigen::Matrix<double, 7, 1> Gest;
    Eigen::Matrix<double, 7, 7> Dest; // Matrice stimata degli attriti
    
    /* Mass Matrix and Coriolis vector with regressor calculation in operative space*/
    Eigen::Matrix<double, 6, 6> MestXi;
    Eigen::Matrix<double, 6, 1> hestXi;
    Eigen::Matrix<double, 6, 6> CestXi;
    Eigen::Matrix<double, 6, 1> GestXi;

    /* Regressor Matrix */
    
    Eigen::Matrix<double, NJ, NJ*PARAM> Y_mod;
    Eigen::Matrix<double, NJ, NJ*PARAM> Y_norm;
    Eigen::Matrix<double, NJ, NJ*FRICTION> Y_D;
    Eigen::Matrix<double, NJ, NJ*FRICTION> Y_D_norm;
	
	/* Pseudo-inverse of jacobian and its derivative matrices */
	Eigen::Matrix<double,6,NJ> J;
    Eigen::Matrix<double,6,NJ> J_dot;
	Eigen::Matrix<double,NJ,6> J_pinv;
    Eigen::Matrix<double,6,NJ> J_T_pinv;
	Eigen::Matrix<double,NJ,6> J_dot_pinv;

    Eigen::Matrix<double,6,NJ> Ja;
    Eigen::Matrix<double,6,NJ> Ja_dot;
	Eigen::Matrix<double,NJ,6> Ja_pinv;
    Eigen::Matrix<double,6,NJ> Ja_T_pinv;
	Eigen::Matrix<double,NJ,6> Ja_dot_pinv;

    Eigen::Matrix<double,NJ,NJ> P; // proiettore nel nullo
    /* Object Regressor Slotine Li*/

    regrob::thunderPanda fastRegMat;

    /* Check the effort limits */
    
    /*Filter function*/
    void aggiungiDato(std::vector<Eigen::Matrix<double, 7, 1>>& buffer_, const Eigen::Matrix<double, 7, 1>& dato_, int lunghezza_finestra_);
    Eigen::Matrix<double, 7, 1> calcolaMedia(const std::vector<Eigen::Matrix<double, 7, 1>>& buffer_);
    double deltaCompute (double a);

    Eigen::Matrix<double, NJ, 1> saturateTorqueRate (
        const Eigen::Matrix<double, NJ, 1>& tau_d_calculated,
        const Eigen::Matrix<double, NJ, 1>& tau_J_d);

    Eigen::Matrix<double, NJ, 1> tau_J_d;

    /* Import parameters */

    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_flag_update_;
    ros::Publisher pub_err_;
    ros::Publisher pub_config_;

    /* Setting Command Callback*/
    
    void setCommandCB(const desTrajEE::ConstPtr& msg);

    /*Setting Flag Callback*/
    void setFlagUpdate(const flag::ConstPtr& msg);

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