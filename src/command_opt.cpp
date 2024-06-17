#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include "panda_controllers/point.h"
// #include "panda_controllers/desTrajEE.h"
#include "panda_controllers/udata.h"
#include "panda_controllers/flag.h"

#include "utils/thunder_panda_2.h"
#include "utils/utils_cartesian.h"

#include "nlopt.hpp"

#ifndef     PARAM
# define    PARAM 10	// number of parameters for each link
#endif

#ifndef     NJ
# define    NJ 7	// number of joints
#endif


/*Variabili inizializzati*/
Eigen::VectorXd q_curr(7);
Eigen::VectorXd dq_curr(7);
Eigen::VectorXd ddq_curr(7);
Eigen::VectorXd H_vec(700);
int l_idx;
int count;


Eigen::Matrix<double, NJ, 1> q_c;
Eigen::Matrix<double, NJ, 1> q_min_limit;
Eigen::Matrix<double, NJ, 1> q_max_limit;
Eigen::Matrix<double, NJ, 1> dq_limit;
Eigen::Matrix<double, NJ, 1> ddq_limit;
std::vector<double>  max_acc(7);
std::vector<double> min_acc(7);


using std::cout;
using std::cin;
using std::endl;

thunder_ns::thunder_panda_2 fastRegMat;

struct UserData {
    // std::vector<double> q;
    // std::vector<double> dq;
    // std::vector<double> ddq;
    // std::vector<std::vector<double>> H;
    std::vector<double> H;
    int l;
    double dt;
} udata;

/*Funzioni necessarie per risoluzione problema di ottimo*/

void jointsCallback( const panda_controllers::udata::ConstPtr& msg );
double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l);


int main(int argc, char **argv)
{

    std::vector<double> lb(NJ), ub(NJ);
    ros::init(argc, argv, "command_opt");
	ros::NodeHandle node_handle;
    double frequency = 1000;
	ros::Rate loop_rate(frequency); 
	
    // udata.q.resize(7);
    // udata.dq.resize(7);
    // udata.ddq.resize(7);
    udata.H.resize(700);
    
    Eigen::Matrix<double, NJ, 1> qr;
    Eigen::Matrix<double, NJ, 1> dot_qr;
    Eigen::Matrix<double, NJ, 1> ddot_qr;
    // q_curr.setZero();
    // l_idx = 11;

    // fastRegMat.init(NJ);

    // q_max_limit << 2.0, 1.0, 2.0, -0.50, 2.50, 3.0, 2.0;
    // q_min_limit << -2.0, -1.0, -2.0, -2.50, -2.0, 0, -2.0;
    q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;
    q_max_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
	q_min_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    dq_limit << 2.1, 2.1, 2.1, 2.1, 2.6, 2.6, 2.6;

    // lbE << -5, -2.5, -5, -5.5, -5, -10, -10;
    // ubE << 5, 2.5, 5, 5.5, 5, 10, 10;
    
	/* Publisher */
	ros::Publisher pub_cmd_opt = node_handle.advertise<sensor_msgs::JointState>("/CT_mod_controller_OS/command_joints_opt", 1);
	ros::Publisher pub_flag_opt = node_handle.advertise<panda_controllers::flag>("/CT_mod_controller_OS/optFlag", 1);

	/* Subscriber */
	ros::Subscriber sub_config = node_handle.subscribe<panda_controllers::udata>("/CT_mod_controller_OS/opt_data", 1, &jointsCallback);

    ros::Time t;
    double t_start;
    double dt = 0;
    // double wf = 20;

    t = ros::Time::now();
    t_start = t.toSec();

    /* Commando di accelerazione*/
    sensor_msgs::JointState command;
    panda_controllers::flag opt_flag_msg;
    command.position.resize(NJ);
    command.velocity.resize(NJ);
    command.effort.resize(NJ);
    qr = q_c;
    dot_qr.setZero();
    ddot_qr.setZero();
    l_idx = 0;
    count = 0;
    std::vector<double> x(NJ), x_old(NJ); // variabile soluzione di ottimo che viene inizializzata
    // std::vector<double> x_old(7);
    for(int i=0; i < 7; ++i){
        x_old[i] = 0;
    }
    
    /*Bound for omega*/
    lb[0] = -M_PI_2; lb[1] = -M_PI_2; lb[2] = -M_PI_2; lb[3] = -M_PI_2; lb[4] = -M_PI_2; lb[5] = -M_PI_2; lb[6] = -M_PI_2;
    ub[0] = M_PI_2; ub[1] = M_PI_2; ub[2] = M_PI_2; ub[3] = M_PI_2; ub[4] = M_PI_2; ub[5] = M_PI_2; ub[6] = M_PI_2;
    // ros::Duration(0.1).sleep();

    while(ros::ok()){
        // ros::spinOnce();
       
        opt_flag_msg.flag = true;

        for(int i=0; i < 7; ++i){
            x[i] = x_old[i];
        }  

        t = ros::Time::now();
        if (dt == 0)
        {
            dt = t.toSec() - t_start;
            cout<<"tempo di inizio:"<<dt;
        }else
        {
            dt = t.toSec() - t_start;
        }
        udata.dt = dt;

        // lbE << -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5;
        // ubE << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5;
        if (count%10 == 0){
            ros::spinOnce();
            
            count = 0;
            for(int i = 0; i < 700; ++i){
                udata.H[i] = H_vec(i);
            }
            udata.l = l_idx;

            nlopt::opt opt(nlopt::algorithm::LN_COBYLA, NJ); // Algoritmo COYBLA -> GRADIENT FRRE OPTIMIZATIO
        

            opt.set_xtol_rel(1e-4);
            opt.set_lower_bounds(lb); // setto limite inferiore
            opt.set_upper_bounds(ub); // setto limite superiore 
            opt.set_min_objective(objective, &udata);  // definisco costo da massimizzare

            // Ottimizzazione
            double minf;
            nlopt::result result = opt.optimize(x, minf);
        }
        count = count+1;

        for(int i = 0; i < 7; ++i){
            qr(i) = q_c(i) + 0.30*sin(x[i]*dt);     
            dot_qr(i) = x[i]*0.30*cos(x[i]*dt);
            ddot_qr(i) = -x[i]*x[i]*0.30*sin(x[i]*dt);      
            x_old[i] = x[i];
            // cout << x[i]<< endl;
        }

        for(int i=0;i<NJ;i++){
            command.position[i] = qr(i);
            command.velocity[i] = dot_qr(i);
            command.effort[i] = ddot_qr(i);
        }

        pub_flag_opt.publish(opt_flag_msg);
        pub_cmd_opt.publish(command);
        loop_rate.sleep();
    }
    return 0;
}

void jointsCallback(const panda_controllers::udata::ConstPtr& msg){
   
    for(int i = 0; i<70; ++i){
        H_vec.segment(i*PARAM, PARAM) << msg->H_stack[i*PARAM], msg->H_stack[i*PARAM+1], msg->H_stack[i*PARAM+2], msg->H_stack[i*PARAM+3], msg->H_stack[i*PARAM+4], msg->H_stack[i*PARAM+5], msg->H_stack[i*PARAM+6], msg->H_stack[i*PARAM+7], msg->H_stack[i*PARAM+8], msg->H_stack[i*PARAM+9];
    }
    l_idx = msg->count;
    // cout<<"valore di ottimo:"<< H_vec;

	// if (!init_start){
	// 	q_start = q_curr;
	// 	init_start = true;
	// }
}

double objective(const std::vector<double> &x, std::vector<double> &grad, void *data){

    UserData *udata = reinterpret_cast<UserData*>(data);
    Eigen::MatrixXd H_true;
    Eigen::VectorXd q(7);
    Eigen::VectorXd dq(7);
    Eigen::VectorXd ddq(7);
    int l;
	double dt;
	// int count;

    H_true.resize(10,70);
    l = udata->l;
    dt = udata->dt;
	// count = udata->count;
	q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;

	if (!grad.empty()) {
        for (int i = 0; i < NJ; i++) {
            grad[i] = 0.0;
        }
    }

	/*Traiettoria sinusoidale ottima*/
    for(int i = 0; i < NJ; ++i){
            q(i) = q_c(i) + 0.30*sin(x[i]*dt);     
            dq(i) = x[i]*0.30*cos(x[i]*dt);
            ddq(i) = -x[i]*x[i]*0.30*sin(x[i]*dt);      
    }
    
    for(int i=0; i<70; ++i){
        H_true.block(0,i,PARAM,1) << udata->H[i*PARAM], udata->H[i*PARAM+1], udata->H[i*PARAM+2], udata->H[i*PARAM+3], udata->H[i*PARAM+4], udata->H[i*PARAM+5], udata->H[i*PARAM+6], udata->H[i*PARAM+7], udata->H[i*PARAM+8], udata->H[i*PARAM+9];
    }
    // cout << H_true.transpose()*H_true<<endl;
	
    fastRegMat.setArguments(q, dq, dq, ddq);
    Eigen::Matrix<double, NJ,PARAM*NJ> Y = fastRegMat.getReg();
    Eigen::Matrix<double, NJ,PARAM> redY = Y.block(0,(NJ-1)*PARAM,NJ,PARAM);

    return redStackCompute(redY, H_true, l);
}


double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l){
    const int P = 10;
    const double epsilon = 0.1;
    double Vmax = 0;
    double cost;

    if (l <= (P-1)){
        if ((red_Y.transpose()-H.block(0,l*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            H.block(0,l*NJ,P,NJ) = red_Y.transpose();
            Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_V(H*H.transpose());
            Vmax = (solver_V.singularValues()).minCoeff();
            cost = 0;
        }
                            
    }else{
        if ((red_Y.transpose()-H.block(0,(P-1)*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            Eigen::MatrixXd Th = H;
            
            Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_V(H*H.transpose());
            double V = (solver_V.singularValues()).minCoeff();

            Eigen::VectorXd S(P);
            for (int i = 0; i < P; ++i) {
                H.block(0,i*NJ,P,NJ) = red_Y.transpose();
                Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_S(H*H.transpose());
                S(i) = (solver_S.singularValues()).minCoeff();
                H = Th;
            }
            Vmax = S.maxCoeff();
            Eigen::Index m; //index max eigvalues 
            S.maxCoeff(&m);
            

            if(Vmax >= V){
                H.block(0,m*NJ,P,NJ) = red_Y.transpose();
            }else{
                Vmax = V;
            }
            Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_cond(H*H.transpose());
            double lmax = (solver_cond.singularValues()).maxCoeff();
            if (Vmax != 0){
                cost = lmax/Vmax;
            }else{
                cost = 0;
            }
            // cout << "ciao"<<endl;
        }
    }
    return cost;
    // return Vmax;
}