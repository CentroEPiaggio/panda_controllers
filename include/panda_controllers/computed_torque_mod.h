#pragma once

#include <array> 
#include <string>
#include <vector>
#include <nlopt.hpp>
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
#include "panda_controllers/udata.h"

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

#ifndef     FRICTION
#define     FRICTION 2	// number of friction parameters for each link
#endif

namespace panda_controllers
{

    class ComputedTorqueMod : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
        hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
    {
  
    public:
    
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
        void starting(const ros::Time&);
        void stopping(const ros::Time&);
        void update(const ros::Time&, const ros::Duration& period);

    private:
    
        bool flag = false;           // flag for check of the desired command velocity
        bool st = true;
        /* Definig the timing */
        
        double dt;
        int l;
       
        ros::Time time_now;
       
        Eigen::Affine3d T0EE;

        /*Variabili filtro Media mobile*/
        std::vector<Eigen::Matrix<double, 7, 1>> buffer_dq; // Array dinamico 7D
        std::vector<Eigen::Matrix<double, 7, 1>> buffer_ddq;
        std::vector<Eigen::Matrix<double, 7, 1>> buffer_tau;
        const int WIN_LEN = 100;


        // Joint (torque, velocity) limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
        
        Eigen::Matrix<double, 7, 1> tau_limit;
        Eigen::Matrix<double, 7, 1> q_dot_limit;
        
        /* Gain Matrices */
        
        Eigen::Matrix<double, 7, 7> Kp; 
        Eigen::Matrix<double, 7, 7> Kv;
        
        Eigen::Matrix<double, 7, 7> Kp_apix; 
        Eigen::Matrix<double, 7, 7> Kv_apix;

        /* Gain for parameters */

        Eigen::Matrix<double, NJ*(PARAM), NJ*(PARAM)> Rinv;
        Eigen::Matrix<double, NJ*(FRICTION), NJ*(FRICTION)> Rinv_fric;
        bool update_param_flag;
    
        /* Defining q_current, dot_q_current, and tau_cmd */

        Eigen::Matrix<double, 7, 1> q_curr;
        Eigen::Matrix<double, 7, 1> dot_q_curr;
        Eigen::Matrix<double, 7, 1> dot_q_curr_old;
        Eigen::Matrix<double, 7, 1> ddot_q_curr;
        Eigen::Matrix<double, 7, 1> ddot_q_curr_old;
        Eigen::Matrix<double, 7, 1> dq_est_old;
        Eigen::Matrix<double, 7, 1> dq_est;
        Eigen::Matrix<double, 7, 1> q_est;
        Eigen::Matrix<double, 7, 1> q_est_old;
        Eigen::Matrix<double, 7, 1> tau_cmd;
        Eigen::Matrix<double, 7, 1> tau_J;
        Eigen::Matrix<double, 7, 1> err_param;
        Eigen::Matrix<double, 7, 1> err_param_frict;
        
        Eigen::MatrixXd H;
        Eigen::VectorXd E;
        Eigen::VectorXd H_vec;
        
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

        /* Mass Matrix and Coriolis vector with regressor calculation*/
        Eigen::Matrix<double, 7, 7> Mest;
        Eigen::Matrix<double, 7, 7> Cest;
        Eigen::Matrix<double, 7, 7> Dest; // Matrice stimata degli attriti
        Eigen::Matrix<double, 7, 1> Gest;
        
        /* Parameter vector */

        Eigen::Matrix<double, NJ*(PARAM), 1> param;    // usfull for calculate an estimate of M, C, G (equivale al pigreco)
        Eigen::Matrix<double, PARAM, 1> param7; 
        Eigen::Matrix<double, NJ*(PARAM), 1> dot_param;
        Eigen::Matrix<double, NJ*(FRICTION), 1> param_frict;
        Eigen::Matrix<double, NJ*(FRICTION), 1> dot_param_frict;
        Eigen::Matrix<double, NJ*(PARAM), 1> param_dyn; 
        Eigen::Matrix<double, NJ*(PARAM+FRICTION), 1> param_tot;

        /* Regressor Matrix */
        
        Eigen::Matrix<double, NJ, NJ*PARAM> Y_mod;
        Eigen::Matrix<double, NJ, NJ*PARAM> Y_norm;
        Eigen::Matrix<double, NJ, PARAM> redY_norm;
        Eigen::Matrix<double, PARAM, 1> redY_stack_sum;
        Eigen::Matrix<double, NJ*PARAM, 1> Y_stack_sum;
        Eigen::Matrix<double, NJ, NJ*FRICTION> Y_D;
        Eigen::Matrix<double, NJ, NJ*FRICTION> Y_D_norm;
        // Eigen::Matrix<double, NJ, NJ*(PARAM+FRICTION)> Y_mod_D;
        // Eigen::Matrix<double, NJ, NJ*(PARAM+FRICTION)> Y_norm_D;

        /* Object Regressor Slotine Li*/

        thunder_ns::thunder_panda_2 fastRegMat;

        /* Check the effort limits */
        
        Eigen::Matrix<double, 7, 1> saturateTorqueRate (
            const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
            const Eigen::Matrix<double, 7, 1>& tau_J_d);
        
        Eigen::Matrix<double, 7, 1> tau_J_d;
        Eigen::Matrix<double, 7, 1> redtau_J;

        static constexpr double kDeltaTauMax {1.0};


        /*Filter function*/
        void aggiungiDato(std::vector<Eigen::Matrix<double, 7, 1>>& buffer_, const Eigen::Matrix<double, 7, 1>& dato_, int lunghezza_finestra_);
        Eigen::Matrix<double, 7, 1> calcolaMedia(const std::vector<Eigen::Matrix<double, 7, 1>>& buffer_);
        double deltaCompute (double a);
        
        /*Function cost*/
        // double objective_function (const std::vector<double> &x, std::vector<double> &grad, void *data);

        double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l, const Eigen::Matrix<double, NJ, 1>& red_tau_J, Eigen::VectorXd& E);

        /* ROS variables */
        
        ros::NodeHandle cvc_nh;
        ros::Subscriber sub_command_;
        ros::Subscriber sub_flag_update_;
        ros::Subscriber sub_flag_opt_;
        
        ros::Publisher pub_err_;
        ros::Publisher pub_config_;
        ros::Publisher pub_opt_;
        
        /* Setting Command Callback*/
        
        void setCommandCB (const sensor_msgs::JointStateConstPtr& msg);
        
        /*Setting Flag Callback*/
        void setFlagUpdate(const flag::ConstPtr& msg);
        

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;


        /* Definizione variabile filtro*/
        // FiltroPassaBasso filtro;

        /* Message */
        
        template <size_t N>
        void fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_);
        void fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_);

        panda_controllers::log_adaptive_joints msg_log;
        panda_controllers::point msg_config;
        panda_controllers::udata msg_opt;
    };

}