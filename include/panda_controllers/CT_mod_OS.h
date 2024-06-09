#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <nlopt.hpp>
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
#include "panda_controllers/rpy.h"
#include "panda_controllers/udata.h"

// #include "utils/ThunderPanda.h"
#include "utils/thunder_panda_2.h"
#include "utils/utils_cartesian.h"
#include "utils/utils_param.h"

#define     DEBUG   0      

#ifndef     NJ
# define    NJ 7	// number of joints
#endif

#ifndef     DOF
# define    DOF 6	// degrees of freedom
#endif

#ifndef     PARAM
# define    PARAM 10	// number of parameters for each link
#endif

#ifndef     FRICTION
#define     FRICTION 2	// number of friction parameters for each link
#endif

// double objective_wrapper(const std::vector<double> &x, std::vector<double> &grad, void *data);

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

    // struct UserData {
    //     std::vector<double> q;
    //     std::vector<double> dq;
    //     // std::vector<std::vector<double>> H;
    //     std::vector<double> H;
    //     int l;
    //     // CTModOS* instance;
    // };

    // double objective(const std::vector<double> &x, std::vector<double> &grad);
    // void solveOptimizationProblem(const UserData &udata);

private:

    bool flag = true;           // flag for check of the desired command velocity    
    double dt;
    int l;
    int t;
    double epsilon; // information trashold 
    int count;

    ros::Time time_now;
    

    /* Robot state handle */
    franka::RobotState robot_state;
    
    /* Franka ROS matrices */
    Eigen::Matrix<double, DOF, NJ> jacobian;
	Eigen::Affine3d T0EE;
    

    // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
    Eigen::Matrix<double, NJ, 1> tau_limit;
    
    Eigen::Matrix<double, NJ, 1> q_min_limit;
    Eigen::Matrix<double, NJ, 1> q_max_limit;
    Eigen::Matrix<double, NJ, 1> dq_limit;
    Eigen::Matrix<double, NJ, 1> ddq_limit;
    
    /* Gain Matrices in OS*/
    Eigen::Matrix<double, DOF, DOF> Lambda; 
    Eigen::Matrix<double, DOF, DOF> Kp; 
    Eigen::Matrix<double, DOF, DOF> Kv;
    Eigen::Matrix<double, DOF, DOF> Kp_xi; 
    Eigen::Matrix<double, DOF, DOF> Kv_xi;

    /* Gain Matrices in CT*/
    Eigen::Matrix<double, NJ, NJ> Kp_j; 
    Eigen::Matrix<double, NJ, NJ> Kv_j; 

    /* Gain Matrice in NullSpacde*/
    Eigen::Matrix<double, NJ, NJ> Kn;

    std:: string robot_name;
    
   
   
     /* Gain for parameters */
    Eigen::Matrix<double, NJ*PARAM, NJ*PARAM> Rinv;
    Eigen::Matrix<double, NJ*(PARAM+FRICTION), NJ*(PARAM+FRICTION)> Rinv_tot;
    Eigen::Matrix<double, NJ*(FRICTION), NJ*(FRICTION)> Rinv_fric;
    bool update_param_flag;
    bool update_opt_flag;


    /* Defining q_current, dot_q_current, s and tau_cmd */
    Eigen::Matrix<double, NJ, 1> q_curr;
    Eigen::Matrix<double, NJ, 1> q_c; // vettore giunti meta corsa
    Eigen::Matrix<double, NJ, 1> dot_q_curr;
    Eigen::Matrix<double, NJ, 1> dot_q_curr_old;
    Eigen::Matrix<double, NJ, 1> ddot_q_curr_old;
    Eigen::Matrix<double, NJ, 1> ddot_q_curr;
    Eigen::Matrix<double, NJ, 1> dot_qr;
    Eigen::Matrix<double, NJ, 1> dot_qr_old;
    Eigen::Matrix<double, NJ, 1> qr;
    Eigen::Matrix<double, NJ, 1> tau_t;
    Eigen::Matrix<double, NJ, 1> tau_d;
    Eigen::Matrix<double, NJ, 1> qr_old;
    Eigen::Matrix<double, NJ, 1> dot_qr_est;
    Eigen::Matrix<double, NJ, 1> ddot_qr;
    Eigen::Matrix<double, NJ, 1> ddot_qr_est;

    /*Variabili di ottimo*/
    Eigen::Matrix<double, NJ, 1> ddq_opt;
    Eigen::Matrix<double, NJ, 1> dq_opt;
    Eigen::Matrix<double, NJ, 1> q_opt;

    Eigen::Matrix<double, NJ, 1> error_q;
    Eigen::Matrix<double, NJ, 1> dot_error_q;
    Eigen::Matrix<double, NJ, 1> dot_error_Nq0;
    
    Eigen::Matrix<double, NJ, 1> err_param;

    Eigen::Matrix<double, NJ, 1> tau_cmd;
    Eigen::Matrix<double, NJ, 1> tau_J;
    Eigen::Matrix<double, NJ, 1> redtau_J;
    // Eigen::Matrix<double, DOF, 1> F_cmd; // Forza commandata agente sull'EE
    Eigen::Matrix<double, DOF, 1> vel_cur;
    
    /* Error and dot error feedback */
    Eigen::Matrix<double, DOF, 1> error;
    Eigen::Matrix<double, DOF, 1> dot_error;
    Eigen::Matrix<double, DOF, 1> dot_error_est;
    Eigen::Matrix<double, DOF, 1> ddot_error;

    Eigen::Vector3d ee_position, ee_velocity, ee_acceleration;
    Eigen::Vector3d ee_omega;

    Eigen::Matrix<double,3,3> ee_rot;
    Eigen::Matrix<double,3,3> Rs_tilde;
    Eigen::Matrix<double,DOF,DOF> L, L_dot;

    /* Used for saving the last command position and command velocity, and old values to calculate the estimation */
    Eigen::Matrix<double, 3, 1> ee_pos_cmd;             // desired command position 
    Eigen::Matrix<double, 3, 1> ee_vel_cmd;             // desired command velocity 
    Eigen::Matrix<double, 3, 1> ee_acc_cmd;             // desired command acceleration 
    
    //Eigen::Matrix<double, 3, 1> ee_ang_cmd;             // desired command position
    Eigen::Matrix<double, 3, 3> ee_rot_cmd;             // desired command position
    Eigen::Matrix<double, 3, 1> ee_ang_vel_cmd;         // desired command velocity 
    Eigen::Matrix<double, 3, 1> ee_ang_acc_cmd;         // desired command acceleration 

    /* FIR variables*/
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_q;
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_dq; // Array dinamico 7D
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_ddq;
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_tau;
    std::vector<Eigen::Matrix<double, 7, 1>> buffer_tau_d;
    std::vector<Eigen::Matrix<double, 6, 1>> buffer_dot_error;
    const int WIN_LEN = 100; // dovrebbe corrispondere a una freq di taglio di circa 100Hz

    /* Parameter vector */
    Eigen::Matrix<double, NJ*PARAM, 1> param;
    Eigen::Matrix<double, PARAM, 1> param7;
    Eigen::Matrix<double, PARAM, 1> param_real;
    Eigen::Matrix<double, NJ*PARAM, 1> param_init;
    Eigen::Matrix<double, NJ*PARAM, 1> dot_param;
    Eigen::Matrix<double, NJ*PARAM, 1> param_dyn;
    Eigen::Matrix<double, NJ*(FRICTION), 1> param_frict;
    Eigen::Matrix<double, NJ*(FRICTION), 1> dot_param_frict;
    Eigen::Matrix<double, NJ*(PARAM+FRICTION), 1> param_tot;
    Eigen::Matrix<double, NJ*(PARAM+FRICTION), 1> param_tot_2;
    Eigen::Matrix<double, NJ*(PARAM+FRICTION), 1> dot_param_tot;

    /* Mass Matrix and Coriolis vector */     
    Eigen::Matrix<double, NJ, NJ> M;
    Eigen::Matrix<double, NJ, 1> C; 
    Eigen::Matrix<double, NJ, 1> G;

    /* Mass Matrix and Coriolis vector with regressor calculation*/
    Eigen::Matrix<double, NJ, NJ> Mest;
    Eigen::Matrix<double, NJ, NJ> Cest;
    Eigen::Matrix<double, NJ, NJ> I7;
    Eigen::Matrix<double, NJ, 1> Gest;
    Eigen::Matrix<double, NJ, 1> Dest; // Friction Matrix
    
    /* Mass Matrix and Coriolis vector with regressor calculation in operative space*/
    Eigen::Matrix<double, DOF, DOF> MestXi;
    Eigen::Matrix<double, DOF, 1> hestXi;
    Eigen::Matrix<double, DOF, DOF> CestXi;
    Eigen::Matrix<double, DOF, 1> GestXi;

    /* Regressor Matrix */
    
    Eigen::Matrix<double, NJ, NJ*PARAM> Y_mod;
    Eigen::Matrix<double, NJ, NJ*(PARAM+FRICTION)> Y_mod_tot;
    Eigen::Matrix<double, NJ, NJ*PARAM> Y_norm;
    Eigen::Matrix<double, NJ, NJ*PARAM> Y_norm_pred;
    Eigen::Matrix<double, NJ, PARAM> redY_norm;
    Eigen::VectorXd redY_norm_vec;
    Eigen::MatrixXd H; // Memory stack
    // Eigen::MatrixXd H_old;
    Eigen::VectorXd H_vec; // utile per passare dati a problema di ottimo
    Eigen::VectorXd E; // Memory stack
    // Eigen::VectorXd E_old;
    Eigen::Matrix<double, NJ*PARAM, 1> Y_stack_sum;
    Eigen::Matrix<double, PARAM, 1> redY_stack_sum;
    Eigen::Matrix<double, 14, 1> redY_stack_sum_fric;
    Eigen::Matrix<double, NJ, NJ*FRICTION> Y_D;
    Eigen::Matrix<double, NJ, NJ*FRICTION> Y_D_norm;
	
	/* Pseudo-inverse of jacobian and its derivative matrices */
	Eigen::Matrix<double,DOF,NJ> J;
    Eigen::Matrix<double,DOF,NJ> J_dot;
	Eigen::Matrix<double,NJ,DOF> J_pinv;
    Eigen::Matrix<double,DOF,NJ> J_T_pinv;
	Eigen::Matrix<double,NJ,DOF> J_dot_pinv;
    
    Eigen::Matrix<double,NJ,NJ> N1; // NullSpace Projector
    Eigen::Matrix<double,NJ,NJ> P; // Projector
    
    /* Object Regressor Slotine Li*/
    // regrob::thunderPanda fastRegMat;
    thunder_ns::thunder_panda_2 fastRegMat;

    /*Variabili utili per approccio braccio reale*/
    Eigen::Matrix<double, NJ, 1> red_tau_J;
    Eigen::Matrix<double, NJ, PARAM> red_Y; 
    // Eigen::VectorXd S;

    /*Filter function*/
    void aggiungiDato(std::vector<Eigen::Matrix<double, NJ, 1>>& buffer_, const Eigen::Matrix<double, NJ, 1>& dato_, int lunghezza_finestra_);
    Eigen::Matrix<double, NJ, 1> calcolaMedia(const std::vector<Eigen::Matrix<double, NJ, 1>>& buffer_);
    // void aggiungiDatoXi(std::vector<Eigen::Matrix<double, DOF, 1>>& buffer_, const Eigen::Matrix<double, DOF, 1>& dato_, int lunghezza_finestra_);
    // Eigen::Matrix<double, DOF, 1> calcolaMediaXi(const std::vector<Eigen::Matrix<double, DOF, 1>>& buffer_);
    
    double deltaCompute (double a);

    /*Cost optimal problem*/
    // double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);

    /* Fuction Stack building*/
    // void stackCompute(const Eigen::Matrix<double, NJ, NJ*PARAM>& Y, Eigen::MatrixXd& H, int& l, const Eigen::Matrix<double, NJ, 1>& tau_J, Eigen::VectorXd& E);
    double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y_new, Eigen::MatrixXd& H,int& l, const Eigen::Matrix<double, NJ, 1>& red_tau_J, Eigen::VectorXd& E);
    // double redStackComputeFric(const Eigen::Matrix<double, NJ, NJ*FRICTION>& red_Y, Eigen::MatrixXd& H,int& l, const Eigen::Matrix<double, NJ, 1>& red_tau_J, Eigen::VectorXd& E);
    // void redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l);

    Eigen::Affine3d computeT0EE(const Eigen::VectorXd& q);
    
    Eigen::Matrix<double, NJ, 1> saturateTorqueRate (
        const Eigen::Matrix<double, NJ, 1>& tau_d_calculated,
        const Eigen::Matrix<double, NJ, 1>& tau_J_d);

    Eigen::Matrix<double, NJ, 1> tau_J_d;

    /* Import parameters */
    static constexpr double kDeltaTauMax {1.0};
    
    /* ROS variables */
    ros::NodeHandle cvc_nh;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_command_j_;
    ros::Subscriber sub_command_rpy_;
    ros::Subscriber sub_flag_opt_;
    ros::Subscriber sub_joints;
    ros::Subscriber sub_flag_update_;
    ros::Publisher pub_err_;
    ros::Publisher pub_config_;
    ros::Publisher pub_opt_;

    /* Setting Command Callback*/
    void setCommandCB(const desTrajEE::ConstPtr& msg);
    void setCommandCBJ(const sensor_msgs::JointStateConstPtr& msg);
    void jointsCallbackT(const sensor_msgs::JointStateConstPtr& msg);
    void setRPYcmd(const rpy::ConstPtr& msg);

    /*Setting Flag Callback*/
    void setFlagUpdate(const flag::ConstPtr& msg);
    void setFlagOpt(const flag::ConstPtr& msg);

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    /* Message */   
    template <size_t N>
    void fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_);
    void fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_);

	panda_controllers::log_adaptive_cartesian msg_log;
    panda_controllers::point msg_config;
    panda_controllers::udata msg_opt;


};

}